import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random

import time
import os
import copy


# def get_rot_matrix(theta_rad):
#     return np.matrix([[np.cos(theta_rad), -np.sin(theta_rad)],[np.sin(theta_rad), np.cos(theta_rad)]])

def get_rot_matrix(theta_deg):
    theta_rad = np.radians(theta_deg) 
    return np.matrix([[np.cos(theta_rad), -np.sin(theta_rad)],[np.sin(theta_rad), np.cos(theta_rad)]])

coord_transform = np.matrix([[0, -1],[1, 0]]) 
# transforms from usual Cartesian coordinates (c) to rosViz coordinates (r)
# x_r = -y_c, y_r = x_p

def rotate_vectors(vx, vy, theta_deg):
    rot = get_rot_matrix(theta_deg)
    vector = np.matrix([[vx], [vy]])
    return np.matmul(rot, vector)

def rotate_vectors(dx, dy, theta):
    dist = np.hypot(dx, dy)
    return np.cos(theta) * dist, np.sin(theta) * dist

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

def gaussian_scalars(stdev, n):
    return np.random.normal(loc=0, scale=stdev, size=n)


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w
    def quick_clone(self):
        p = Pose()
        p.position = Point()
        p.position.x = self.pose.position.x
        p.position.y = self.pose.position.y
        p.position.z = self.pose.position.z
        p.orientation = Quaternion()
        p.orientation.x = self.pose.orientation.x
        p.orientation.y = self.pose.orientation.y
        p.orientation.z = self.pose.orientation.z
        p.orientation.w = self.pose.orientation.w
        return Particle(p,self.w)

class ParticleFilter:
    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        # rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()
        self.map_set = False

        print("HERE")
        self.closestMap = np.load("computeMap.npy")

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
            # pcloud is only updated at the end of each scan cycle
        self.particle_cloud = np.array([Particle(Pose(),0) for _ in range(self.num_particles)]) 
        # self.particle_cloud = []
            # the bulk of the computation is performed on the following:
        self.poses = np.zeros(shape=(2, self.num_particles)) # x, y only
        self.yaws = np.zeros(shape=self.num_particles) # yaws only
        self.weights = np.ones(shape=self.num_particles)
        
        
        self.poses_resample = np.zeros(shape=(2, self.num_particles)) # x, y only
        self.yaws_resample = np.zeros(shape=self.num_particles) # yaws only
        self.weights_resample = np.ones(shape=self.num_particles)

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        # self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        # self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        # rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        # rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        # self.tf_listener = TransformListener()
        # self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True
        
    def initialize_particle_cloud(self):
        
        w = 384
        h = 384
        
        assert (w != 0)
        assert (h != 0)
        
        map_res = 0.05
        map_origin_x = -10
        map_origin_y = -10
        
        rng = np.random.default_rng()
        for i in range(self.num_particles):
            #print(self.map.info)
            x = rng.integers(low=0,high=w)
            y = rng.integers(low=0,high=h)
            yaw = rng.random() * 2 * np.pi
            
            # while self.map.data[y*w + x] > 10 or self.map.data[y*w+x] == -1:
            #     x = rng.integers(low=0,high=w)
            #     y = rng.integers(low=0,high=h)
            
            adj_x = x * map_res + map_origin_x
            adj_y = y * map_res + map_origin_y
            self.poses[:, i] = [adj_x,adj_y]
            self.yaws[i] = yaw
        
        self.update_particle_cloud()


        #self.particle_cloud = self.particle_cloud
        self.normalize_particles()
    
    def normalize_particles(self):
        self.weights /= np.sum(self.weights)
        sum_weight = np.sum(self.weights)
        float_error = 1 - sum_weight
        if float_error >= 0:
            self.particle_cloud[0].w += float_error
            return
        
        for i in range(self.num_particles):
            if float_error < -self.weights[i]:
                self.weights[i] = 0
                float_error += self.weights[i]
                continue
            else:
                self.weights[i] += float_error
                return
        
        assert(False) # this should be unreachable
        
        
        # make all the particle weights sum to 1.0
        total_weight = 0
        sum_weight = 0
        for particle in self.particle_cloud:
            if math.isnan(particle.w):
    
                particle.w = 0
            total_weight += particle.w

        assert (total_weight > 0)
        for particle in self.particle_cloud:
            
            particle.w /= total_weight
            sum_weight += particle.w
        # float error processing
        float_error = 1 - sum_weight
        if float_error >= 0:
            self.particle_cloud[0].w += float_error
            return
        
        for particle in self.particle_cloud:
            if float_error < -particle.w:
                particle.w = 0
                float_error += particle.w
                continue
            else:
                particle.w += float_error
                return
        
        assert(False) # this should be unreachable
    
    def update_particles_with_motion_model(self):        
        angle_increment = 15 * np.pi / 180
        linear_increment = 0.05
        use_dummy_deltas = True
        

        if use_dummy_deltas:
            dx_raw = 2
            dy_raw = 3
            dyaw_raw = 25
        else:
            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
            
            dx_raw = curr_x - old_x
            dy_raw = curr_y - old_y
            dyaw_raw = curr_yaw - old_yaw
            # print(dx_raw, dy_raw, dyaw_raw)
        
        dx_noise = gaussian_scalars(linear_increment, self.num_particles)
        dy_noise = gaussian_scalars(linear_increment, self.num_particles)
        dyaw_noise = gaussian_scalars(angle_increment, self.num_particles)
        
        dxs = dx_noise + dx_raw
        dys = dy_noise + dy_raw
        dyaws = dyaw_noise + dyaw_raw
        
        
        self.yaws = (self.yaws + dyaws) % (2 * np.pi)
        dists = np.hypot(dxs, dys)
        trans_dxs = dists * np.cos(self.yaws) 
        trans_dys = dists * np.sin(self.yaws) 
        self.poses[0, :] += trans_dxs
        self.poses[1, :] += trans_dys
        
        
        # for i in range(self.num_particles):
        #     pcle = self.particle_cloud[i]
            
        #     orient = pcle.pose.orientation
        #     euler_angles = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        #     old_pyaw = euler_angles[2]
        #     new_pyaw = (euler_angles[2] + dyaws[i]) % (2 * np.pi)
        #     new_q = quaternion_from_euler(0, 0, new_pyaw)
        #     pcle.pose.orientation.x = new_q[0]
        #     pcle.pose.orientation.y = new_q[1]
        #     pcle.pose.orientation.z = new_q[2]
        #     pcle.pose.orientation.w = new_q[3]
            
        #     dx_t, dy_t = rotate_vectors(dxs[i], dys[i], new_pyaw)
        #     pcle.pose.position.x += dx_t
        #     pcle.pose.position.y += dy_t

    
    
    def update_particle_weights_with_measurement_model(self, data):
        deltaArr = []
        sigma_hit = .1
        z_max = .1
        z_hit = .89
        z_random = 0.01
        enable_measure_model_demo = True

        
        angle_incr = 3 # 3 increment ~ 0.9 degrees
        angle_indices = np.arange(0, 1147, angle_incr)
        num_angles = len(angle_indices)
        
        lidar_angles = angle_indices * (2 * np.pi) / 1147 - np.pi
        
        if enable_measure_model_demo:
            # ranges = np.ones(num_angles)
            ranges = gaussian_scalars(0.5, num_angles) + 1.5
        else:
            ranges = np.array(data.ranges)[angle_indices]
        ranges[ranges == float("inf")] = 12
        
        
        width = 384
        height = 384
        map_res = 0.05
        pos_x = -10
        pos_y = -10
        
        
        # how much we multiply each pcle weight subcomponent so that the 
        # product doesn't zero out
        preserve_factor = 1.2 
        
        # default penalty if a transformed scan point falls out of the map
        oob_penalty = 20
        
                
        for i in range(self.num_particles):
            # t1 = time.perf_counter()
            curr_yaw = self.yaws[i]
            trans_dxs = ranges * np.cos(lidar_angles + curr_yaw)
            trans_dys = ranges * np.sin(lidar_angles + curr_yaw)
            
            norm_xs = ((trans_dxs + self.poses[0, i] - pos_x)/map_res).astype(int)
            norm_ys = ((trans_dys + self.poses[1, i] - pos_y)/map_res).astype(int)
            
            dists = np.zeros(num_angles) + oob_penalty
            in_bound = (norm_xs >= 0) & (norm_xs < width) & (norm_ys >= 0) & (norm_ys < height)
            dists = self.closestMap[norm_xs[in_bound], norm_ys[in_bound]]
            # x_oob_penalty = np.where(norm_xs < 0, -norm_xs, 0) + np.where(norm_xs >= width, norm_xs - width, 0)
            # y_oob_penalty = np.where(norm_ys < 0, -norm_ys, 0) + np.where(norm_ys >= height, norm_ys - width, 0)
            # dists += (x_oob_penalty + y_oob_penalty)
            
            zr_zm = 0
            if z_max != 0:
                zr_zm = z_random/z_max
            
            prob_ds = np.exp(-pow(dists,2)/(2 * pow(sigma_hit,2))) / (sigma_hit * np.sqrt(2 * np.pi))
            
            self.weights[i] = np.prod(preserve_factor * z_hit * prob_ds + zr_zm)
        
        return
    
    def update_estimated_robot_pose(self):        
        av_x = np.average(self.poses[0, :])
        av_y = np.average(self.poses[1, :])
        av_yaw = np.average(self.yaws)
        
        pose = self.robot_estimate
        # pose.position = Point()
        pose.position.x = av_x
        pose.position.y = av_y
        pose.orientation = quaternion_from_euler(0,0,av_yaw)
    
    def update_particle_cloud(self):
        "Applies latest values in poses, yaws, and weights to particle_cloud."
        for i in range(self.num_particles):
            pcle = self.particle_cloud[i]
            pcle.pose.position.x = self.poses[0, i]
            pcle.pose.position.y = self.poses[1, i]
            new_q = quaternion_from_euler(0, 0, self.yaws[i])
            pcle.pose.orientation.z = new_q[2]
            pcle.pose.orientation.w = new_q[3]
            pcle.w = self.weights[i]
    

    def resample_particles(self):
        from collections import Counter
        counts = Counter(np.random.choice(np.arange(self.num_particles), size=self.num_particles, p=self.weights))
        i = 0
        for index in counts.keys():
            n = counts[index]
            self.poses_resample[:, i:i+n] = self.poses[:, index][:, np.newaxis]
            self.yaws_resample[i:i+n] = self.yaws_resample[index]
            self.weights_resample[i:i+n] = self.weights[index]
            i += n
        poses_tmp = self.poses
        yaws_tmp = self.yaws
        weights_tmp = self.weights
        self.poses = self.poses_resample
        self.yaws = self.yaws_resample
        self.weights = self.weights_resample
        self.poses = poses_tmp
        self.yaws = yaws_tmp
        self.weights = weights_tmp
        return
    
        sample = np.random.choice(self.particle_cloud, size=self.num_particles, p=self.weights)
        
        encountered = set()
        for i in range(self.num_particles):
            pcle = sample[i]
            if pcle in encountered:
                self.particle_cloud[i] = pcle.quick_clone()
            else:
                self.particle_cloud[i] = pcle
                encountered.add(pcle)
        return
         

import timeit
if __name__=="__main__":
    
    # print(rotate_vectors(0, 1, 45))
    pf = ParticleFilter()
    # struct_array = np.array([tuple(x) for x in self.particle_cloud], dtype=list(zip(np.arange(self.num_particles), s)))
    
    
    start = time.perf_counter()
    pf.update_particles_with_motion_model()
    end = time.perf_counter()
    print("motion_model: ", end - start)
    
    start = time.perf_counter()
    pf.update_particle_weights_with_measurement_model(None)
    end = time.perf_counter()
    print("measure_model: ", end - start)
    
    start = time.perf_counter()
    pf.normalize_particles()
    end = time.perf_counter()
    print("normalize: ", end - start)
    
    pf.update_particle_cloud()
    
    start = time.perf_counter()
    pf.resample_particles()
    end = time.perf_counter()
    print("resample: ", end - start)
    
    start = time.perf_counter()
    pf.update_estimated_robot_pose()
    end = time.perf_counter()
    print("update_pose: ", end - start)
    
    start = time.perf_counter()
    pf.update_particle_cloud()
    end = time.perf_counter()
    print("update_pcloud: ", end - start)
    
    
    
    
    exit(0)
    xs = [0,-1,0,1]
    ys = [1,0,-1,0]
    cardinal_dirs = np.matrix([xs, ys]) # N, W, S, E
    
    rot = get_rot_matrix(0)
    
    rot_dirs = np.matmul(rot, cardinal_dirs)
    rot_then_trans = np.matmul(coord_transform, rot_dirs)
    
    print(rot_dirs)
    print(rot_dirs[:,0])
    print(rot_dirs.T)
    print("\n\n")
    print(rot_then_trans)
    print(rot_then_trans[0,:])
    print(rot_then_trans.T)
    print("\n\n")
    print(np.matmul(coord_transform, rot))