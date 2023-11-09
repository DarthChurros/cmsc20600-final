#!/usr/bin/env python3

'''
***
LIDAR variant: RPLIDAR-A1.
***
'''

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
from geometry_msgs.msg import Twist, Vector3

# demo toggles
enable_motion_demo = False
enable_closestMap_viz_demo = False

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
    '''
    Helper to generate a normally-distributed random tape.
    
    Returns an n-size array of floats sampled from a normal distribution
    with mean 0 and standard deviation stdev.
    '''
    return np.random.normal(loc=0, scale=stdev, size=n)

def absolute_cutoff(x, limit):
    '''
    Ensure that x has a magnitude of at most limit, rounding to the nearest
    interval of [-limit, limit] if necessary.
    
    Takes:
    - a value x
    - limit > 0
    
    Outputs:
    - If x falls within [-limit, limit], return x
    - Else, round x to the nearest endpoint of [-limit, limit].
    '''
    assert (limit > 0)
    if x > limit:
        # x greater than limit, floor to limit
        return limit
    elif x < -limit:
        # x less than -limit, ceil to -limit
        return -limit
    else:
        # x in bound, return as is
        return x




class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w


def demo_visualize_closestMap(self):
    '''
    Demo: plots the closestMap as a heat map in matplotlib and exits.
    '''
    closestMap = self.closestMap
    while not rospy.is_shutdown():
        if self.map_set:
            break
    arr_map = np.array(self.map.data)
    w = self.map.info.width
    h = self.map.info.height
    in_maze = (arr_map != -1).reshape(w, -h).T
    assert (in_maze.shape == (384, 384))
    
    import sys
    import matplotlib.pyplot as plt
    # for visualizing maze
    # cutoff = 0.6
    # closestMap[closestMap >= cutoff] = cutoff
    
    # for visualizing open paths
    cutoff = 0.157 # max distance that leaves at least one line of pixel in each path
    above_cutoff = closestMap >= cutoff
    closestMap[above_cutoff & in_maze] = 0
    closestMap[~in_maze] = cutoff
    
    plt.imshow(closestMap, cmap='hot', interpolation='nearest')
    plt.show()
    exit(0)
    
    
    
    
def add_node(path_arr, dx, dy, idx):
    assert (idx >= 1)
    if idx == path_arr.shape[1]:
        return -1
    
    path_arr[:, idx] = path_arr[:, idx - 1] + [dx, dy, 0]
    path_arr[2:, idx - 1] = np.arctan2(dy, dx)
    print("idx: ",idx ,path_arr[:, idx])
    return idx + 1


def init_pose(pose, x, y, yaw):
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    q = quaternion_from_euler(0,0, yaw)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    
def halt(publisher):
    '''Halts the robot. i.e. publishes a 0 Twist to cmd_vel'''
    publisher.publish(Twist(linear=Vector3(0,0,0),angular=Vector3(0,0,0)))


def wrapto_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
    # https://stackoverflow.com/questions/15927755/opposite-of-numpy-unwrap
    
    angle %= (2 * np.pi)
    assert (-2 * np.pi <= angle <= 2 * np.pi)
    
    if angle <= -np.pi:
        return 2 * np.pi + angle
    elif angle > np.pi:
        return angle - 2 * np.pi
    else:
        return angle



class ParticleFilter:
    def publish_pose(self):
        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.curr_pose
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()
        self.map_set = False # whether map has been set

        print("HERE")
        # load closestMap array
        self.closestMap = np.load("computeMap.npy") 

        # the number of particles used in the particle filter
        self.num_particles = 1000

        # initialize the particle cloud array
        self.particle_cloud = np.array([Particle(Pose(),0) for _ in range(self.num_particles)]) 
        
        
        '''
        Initialize the internal particle state variables.
        The bulk of the computations are performed on these variables, and their
        states propagated to particle_cloud just before publishing.
        
        poses = [[x_0, x_1, x_2,...], [y_0, y_1, y_2,...]]
        - poses[0, :] = [x_0, x_1, x_2,...]
        - poses[1, :] = [y_0, y_1, y_2,...]
        - poses[:, i] = [x_i, y_i]
        - x_i, y_i = x and y positions of particle i respectively
        
        yaws = [yaw_0, yaw_1, yaw_2,...]
        - yaw_i = yaw of particle i
        
        weights = [w_0, w_1, w_2,...]
        - w_i = weight of particle i
        '''
        self.poses = np.zeros(shape=(2, self.num_particles)) # x, y of poses only
        self.yaws = np.zeros(shape=self.num_particles) # yaws only
        self.weights = np.ones(shape=self.num_particles)
        
        '''
        Resample variants of the internal particle state variables. 
        Used as buffers to temporarily store resampled values during resampling.
        
        "X_resample" is structurally identical to "X"
        '''
        self.poses_resample = np.zeros(shape=(2, self.num_particles)) # x, y only
        self.yaws_resample = np.zeros(shape=self.num_particles) # yaws only
        self.weights_resample = np.ones(shape=self.num_particles)

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        # self.lin_mvmt_threshold = 0.2        
        # self.ang_mvmt_threshold = (np.pi / 6)
        # self.lin_mvmt_threshold = 0.01        
        # self.ang_mvmt_threshold = (np.pi / 180)
        self.lin_mvmt_threshold = 0        
        self.ang_mvmt_threshold = 0
        self.first_scan_rcvd = True # on the first received scan, we should check deltas (otherwise it will never start)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)
        
        # our addition:
        if (enable_closestMap_viz_demo):
            demo_visualize_closestMap(self) # input the whole cloestMap
            # demo_visualize_closestMap(self.closestMap[190:300 + 1, 160:300 + 1])

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()
        
        path = np.zeros(shape=(3,6))
        path[:, 0] = [0.8, 0.3, 0]
        idx = 1
        # idx = add_node(path, 1, 1, idx)
        # idx = add_node(path, 1, 2, idx)
        # idx = add_node(path, 2, 1, idx)
        # idx = add_node(path, 0, -2, idx)
        # idx = add_node(path, -2, -2, idx)
        idx = add_node(path, 0.25, 1.15, idx)
        idx = add_node(path, 0.45, 0, idx)
        idx = add_node(path, -0.12, -1.3, idx)
        # idx = add_node(path, 1.5, 1, idx)
        # idx = add_node(path, 1, 0.5, idx)
        # idx = add_node(path, 0, -1, idx)
        # idx = add_node(path, -1, -1, idx)
        print(path)
        self.path = path
        self.node_idx = 0
        
        
        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        for i in range(path.shape[1]):
            curr_pose = Pose()
            init_pose(curr_pose, path[0, i], path[1, i], path[2, i])
            particle_cloud_pose_array.poses.append(curr_pose)
            
        self.curr_pose = Pose()
        init_pose(self.curr_pose, 0.75, 0.25, 0)
        self.publish_pose()

        for i in range(10):
            self.particles_pub.publish(particle_cloud_pose_array)
            time.sleep(0.1)
        
        self.path_idx = 0
        self.target_yaw = self.path[2, self.path_idx]
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # initialize shutdown callback so that robot halts on ctrl + c:
        rospy.on_shutdown(lambda : halt(self.pub_cmd_vel))
        



        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True
        return



    def get_map(self, data):

        self.map = data
        self.map_set = True
    

    def demo_motion_model(self):
        '''
        Demo: updates particles with dummy (constant) delta values and publishes
        to rVIZ at a fixed frequency
        '''
        
        update_num = 1
        time_step = 0.2
        while not rospy.is_shutdown():
            print(f"Start sleep! ({update_num})")
            for i in range(5):
                # sleep for 5 time steps
                time.sleep(time_step)
                print("... ", round(time_step * (i + 1), 1))
            print(f"End sleep! ({update_num})")
            # update particles by dummy deltas
            self.update_particles_with_motion_model()
            # publish updated particles
            self.publish_particle_cloud()
            update_num += 1
        exit(0)
        
    
    def update_particle_cloud(self):
        '''Updates particle_cloud with latest poses, yaws, and weights
        encoded in internal state arrays.'''
        for i in range(self.num_particles):
            # get particle pointer
            pcle = self.particle_cloud[i]
            
            # update position of particle
            pcle.pose.position.x = self.poses[0, i] # poses[0, i] = x_i
            pcle.pose.position.y = self.poses[1, i] # poses[0, i] = y_i
            # z position should not change, therefore it needs no update
            
            # update orientation of particle
            new_q = quaternion_from_euler(0, 0, self.yaws[i])
            pcle.pose.orientation.x = new_q[0]
            pcle.pose.orientation.y = new_q[1]
            pcle.pose.orientation.z = new_q[2]
            pcle.pose.orientation.w = new_q[3]
            
            # update weight of particle
            pcle.w = self.weights[i]

    def initialize_particle_cloud(self):
        '''
        Initialize the particle cloud with randomly generated positions and yaws.
        
        Updates are propagated to BOTH particle_cloud and internal state arrays.
        '''
        
        while (not self.map_set):
            # wait until the map is set, you idiot
            time.sleep(0.1)

        # extract map info
        w = self.map.info.width
        h = self.map.info.height
        map_res = self.map.info.resolution
        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y
        
        # map dimensions should be nonzero
        assert (w != 0)
        assert (h != 0)
        
        rng = np.random.default_rng()
        
        # generate random particles
        for i in range(self.num_particles):
            # generate random position and yaw for new particle
            x = rng.integers(low=0,high=w)
            y = rng.integers(low=0,high=h)
            yaw = rng.random() * 2 * np.pi
            
            # while self.map.data[y*w + x] > 10 or self.map.data[y*w+x] == -1 or x >= 223:
            while self.map.data[y*w + x] > 10 or self.map.data[y*w+x] == -1:
                # re-generate (x,y) until it no longer:
                # 1) occupies an obstacle, OR
                # 2) is out of bounds
                x = rng.integers(low=0,high=w)
                y = rng.integers(low=0,high=h)
            
            # convert (x,y) to map coordinates
            adj_x = x * map_res + map_origin_x
            adj_y = y * map_res + map_origin_y
            
            # set particle internal state arrays at corresponding particle position
            self.poses[:, i] = [adj_x,adj_y] # poses[:, i] = [x_i, y_i]
            self.yaws[i] = yaw               # yaws[i] = [yaws_i]
        
        # apply internal states to particle_cloud
        self.update_particle_cloud()
        
        # normalize particle weights
        self.normalize_particles()


        # publish particle cloud
        num_publishes = 10
        for i in range(num_publishes):
            '''
            Strangely, if you do not publish multiple times during initialization, 
            small (< 1000 particles) particle clouds do not show up in rViz.
            '''
            self.publish_particle_cloud()
            time.sleep(0.1)
        
        
        
        if (enable_motion_demo):
            # execute demo if enabled
            self.demo_motion_model()


    def normalize_particles(self):
        '''
        Normalize particle weights
        
        WARNING: only updates internal state array, NOT particle_cloud
        '''
        self.weights /= np.sum(self.weights)
        
        
        
        '''
        Float error processing:
        Due to the large number of weights involved, float error is a concern. 
        
        If the normalized weights do not add up to 1, invoking np.random.choice 
        on these weights will fail.
        '''
        sum_weight = np.sum(self.weights)
        float_error = 1 - sum_weight
        if float_error >= 0:
            # sum_weight <= 1 : must increase sum_weight to 1
            
            # simply add float_error to one of the weights
            self.particle_cloud[0].w += float_error
            return
        
        for i in range(self.num_particles):
            # sum_weight > 1 : must decrease sum_weight to 1
            
            if float_error < -self.weights[i]:
                '''
                Adding float_error to this weight will make the weight negative,
                which is invalid.
                
                Add only as much of the float error we can (zeroing the weight)
                and move on to the next weight.
                '''
                
                self.weights[i] = 0
                float_error += self.weights[i]
                continue
            else:
                '''
                Adding float_error to this weight will make it non-negative,
                which is valid.
                
                Add the float_error to the weight and exit.
                '''
                
                self.weights[i] += float_error
                return
            
            
        '''
        This should be unreachable.
        
        If execution reaches here, then it means we couldn't compensate 
        for the float error by adjust each weight.
        '''
        
        assert(False)




    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses 

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        '''
        Resample particles in accordance with weights.
        
        WARNING: only updates internal state arrays, NOT particle_cloud
        '''
        
        from collections import Counter
        
        # sample particle (indices) in accordance with weights
        sample = np.random.choice(np.arange(self.num_particles), size=self.num_particles, p=self.weights)
        
        '''
        Count sampling frequencies
        counts[i] = number of times particle i has been included in the new sample
        '''
        counts = Counter(sample)
        
        dst = 0 # current destination index to fill in resample buffers
        i = 0 # particle index in sample
        for i in counts.keys():
            n = counts[i]
            
            # fill n-long stretch in the buffer with the state of particle i
            # this simulates picking n copies of particle i
            self.poses_resample[:,dst:dst+n] = self.poses[:, i][:, np.newaxis]
            self.yaws_resample[dst:dst+n] = self.yaws[i]
            self.weights_resample[dst:dst+n] = self.weights[i]
            
            # advance to the next available stretch in the buffer
            dst += n
        
        # the resample buffer is now the current particle internal state 
        # ...so swap normal and resample buffers
        # (tmps are used as temporary storage to avoid overwriting existing values)
        poses_tmp = self.poses
        yaws_tmp = self.yaws
        weights_tmp = self.weights
        self.poses = self.poses_resample
        self.yaws = self.yaws_resample
        self.weights = self.weights_resample
        self.poses_resample = poses_tmp
        self.yaws_resample = yaws_tmp
        self.weights_resample = weights_tmp
        return



    def robot_scan_received(self, data):
        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if len(self.particle_cloud) > 0:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold or
                self.first_scan_rcvd):
                self.first_scan_rcvd = False
                
                curr_x = self.curr_pose.position.x
                curr_y = self.curr_pose.position.y
                curr_yaw = get_yaw_from_pose(self.curr_pose)
                
                next_node = self.path[:2, self.node_idx + 1]
                d = np.hypot(next_node[0] - curr_x, next_node[1] - curr_y)
                if d < 0.02:
                    self.node_idx += 1
                    next_node = self.path[:2, self.node_idx + 1]
                
                target_yaw = np.arctan2(next_node[1] - curr_y, next_node[0] - curr_x)
                ang_error = wrapto_pi(target_yaw - curr_yaw)

                
                deltas = self.path[:2, self.node_idx + 1] - [curr_x, curr_y]
                dist = np.hypot(deltas[0], deltas[1])
                # print(f"curr {self.node_idx}, target {self.node_idx + 1}")
                # print(f"target_yaw {np.rad2deg(target_yaw)}, deltas {deltas}, dist {dist}")
                
                # lin_error = absolute_cutoff(dist + 0.4, limit=1) * (abs(ang_error) / np.pi - 1) ** 12
                lin_error = 0.4 * (abs(ang_error) / np.pi - 1) ** 12
                
                self.pub_cmd_vel.publish(Twist(linear=Vector3(0.2*lin_error,0,0),angular=Vector3(0,0,ang_error)))
                init_pose(self.curr_pose, curr_x, curr_y, curr_yaw)
                self.publish_pose()
                self.odom_pose_last_motion_update = self.odom_pose
                
                # return                

                # This is where the main logic of the particle filter is carried out

                print("START ====")                
                t = time.time()
                self.update_particles_with_motion_model()
                print("motion", time.time()-t)
                
                t = time.time()
                self.update_particle_weights_with_measurement_model(data)
                print("measure", time.time()-t)
                
                t = time.time()
                self.normalize_particles()
                print("norm", time.time()-t)

                t = time.time()
                self.resample_particles()
                print("resample", time.time()-t)
                
                # apply internal states to particle buffer
                self.update_particle_cloud()

                t = time.time()
                self.update_estimated_robot_pose()
                print("update pose", time.time()-t)

                t = time.time()
                self.publish_particle_cloud()
                print("pub cloud", time.time()-t)

                t = time.time()
                self.publish_estimated_robot_pose()
                print("pub pose", time.time()-t)
                print("END ====\n")                

                # diffs = np.array(starts) - np.array(ends)
                # print("diffs", diffs)

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        ''' Update the estimated robot pose. '''
        
        # Compute average position and yaw (weighted with particle weights).
        av_x = np.average(self.poses[0, :], weights=self.weights)
        av_y = np.average(self.poses[1, :], weights=self.weights)
        av_yaw = np.average(self.yaws, weights=self.weights)
        
        # Update current robot pose to computed averages
        pose = self.robot_estimate
        # pose.position = Point()
        pose.position.x = av_x
        pose.position.y = av_y
        pose.position.z = 0
        # Need to convert yaw to quaternion
        quat = quaternion_from_euler(0,0,av_yaw)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        init_pose(self.curr_pose, av_x, av_y, av_yaw)
        return


    def update_particle_weights_with_measurement_model(self, data):
        '''
        Set particle weights to likelihood field probability (products) computed 
        from latest scan and odom data.
        
        WARNING: only updates internal state array, NOT particle_cloud
        '''
        
        # monte carlo PF algo constants
        sigma_hit = .1
        z_max = .1
        z_hit = .89
        z_random = 0.01
        
        # demo toggle
        enable_measure_model_demo = False

        # Include every "angle_incr"-th lidar angle
        angle_incr = 1 # 3 increment ~ 0.9 degrees
        angle_indices = np.arange(0, 360, angle_incr)
        num_angles = len(angle_indices)
        
        # convert from lidar angle indices to angles
        lidar_angles = angle_indices * (2 * np.pi) / 360
        
        if enable_measure_model_demo:
            # dummy ranges value for demo purposes
            ranges = gaussian_scalars(0.5, num_angles) + 1.5
        else:
            # get ranges only for included angle_indices
            ranges = np.array(data.ranges)[angle_indices]
        
        # preprocess; set all infinite values to max distance
        ranges[ranges == float("inf")] = 12
        
        # extract map info
        width = self.closestMap.shape[1]
        height = self.closestMap.shape[0]
        map_res = self.map.info.resolution
        pos_x = self.map.info.origin.position.x
        pos_y = self.map.info.origin.position.y
        
        
        # how much we multiply each pcle weight subcomponent so that the 
        # product doesn't zero out
        preserve_factor = 1.2 
        
        # default distance penalty if a transformed scan point falls out of the map
        oob_penalty = 20
        
        for i in range(self.num_particles):
            # yaw of ith particle
            curr_yaw = self.yaws[i]
            
            # x,y positions of ith particle
            x_i = self.poses[0, i]
            y_i = self.poses[1, i]
            
            # reorient "ranges" vectors to particle yaw...
            trans_dxs = ranges * np.cos(lidar_angles + curr_yaw)
            trans_dys = ranges * np.sin(lidar_angles + curr_yaw)
            
            # ...then translate to particle position
            trans_dxs = trans_dxs + x_i
            trans_dys = trans_dys + y_i
            
            # convert transformed ranges to closestMap indices
            norm_xs = ((trans_dxs - pos_x)/map_res).astype(int)
            norm_ys = ((trans_dys - pos_y)/map_res).astype(int)
            
            # closest obstacle distances; initialized to out-of-bound penalty
            # (until proven to be "in_bound", assume all "ranges" are out-of-bound)
            dists = np.zeros(num_angles) + oob_penalty
            
            # boolean mask indicating all in-bound range indices
            in_bound = (norm_xs >= 0) & (norm_xs < width) & (norm_ys >= 0) & (norm_ys < height)
            # inititalize only in-bound closest map distances
            dists = self.closestMap[norm_xs[in_bound], norm_ys[in_bound]]
            
            zr_zm = 0
            if z_max != 0:
                # set zr_zm IFF z_max is nonzero
                zr_zm = z_random/z_max
            
            # compute likelihood probabilities of each distance
            prob_ds = np.exp(-pow(dists,2)/(2 * pow(sigma_hit,2))) / (sigma_hit * np.sqrt(2 * np.pi))
            
            # set particle weight with product of probabilities
            self.weights[i] = np.prod(preserve_factor * z_hit * prob_ds + zr_zm)


        

    def update_particles_with_motion_model(self):        
        '''
        Updates particles' positions and yaws by the latest movement of the bot.
        
        WARNING: only updates internal state arrays, NOT particle_cloud
        '''
        
        # standard deviations for noise
        angle_increment = 15 * np.pi / 180
        linear_increment = 0.1
        
        # demo toggle
        use_dummy_deltas = enable_motion_demo
        
        if use_dummy_deltas:
            # dummy delta values for demo purposes
            dx_raw = 2
            dy_raw = 3
            dyaw_raw = 25
        else:
            # get current and last update positions/yaws
            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
            
            # compute positions/yaws deltas (i.e. changes)
            dx_raw = curr_x - old_x
            dy_raw = curr_y - old_y
            dyaw_raw = curr_yaw - old_yaw
        
        # generate random tapes for movement noise
        # e.g. dx_noise[i] == x translation noise for particle i
        dx_noise = gaussian_scalars(linear_increment, self.num_particles)
        dy_noise = gaussian_scalars(linear_increment, self.num_particles)
        dyaw_noise = gaussian_scalars(angle_increment, self.num_particles)
        
        # compute deltas for each particle, adjusted by noise
        dxs = dx_noise + dx_raw
        dys = dy_noise + dy_raw
        dyaws = dyaw_noise + dyaw_raw
        
        # update particle yaws by corresponding deltas (modulo to wrap around)
        self.yaws = (self.yaws + dyaws) % (2 * np.pi)
        
        # reorient position deltas to particle yaws
        dists = np.hypot(dxs, dys)
        trans_dxs = dists * np.cos(self.yaws) 
        trans_dys = dists * np.sin(self.yaws) 
        
        # update particle positions by corresponding (transformed) deltas
        self.poses[0, :] += trans_dxs
        self.poses[1, :] += trans_dys            
        
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # TODO



if __name__=="__main__":
    

    pf = ParticleFilter()
    print("HERE")
    rospy.spin()









