#!/usr/bin/env python3

'''
***
LIDAR variant: RPLIDAR-A1.
***
'''

import rospy

from turtlebot3_msgs.msg import Sound
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped, Twist, Vector3
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
import threading

import time
import os
import copy

from motion import Motion

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


class PathFinding:

    def __init__(self, map, start, destination, algorithm="dijstra", bound=4):
        self.algorithm = algorithm
        self.map = map
        self.path = []
        self.current_pose = start
        self.destination = destination
        self.bound = bound

    #sets the postition of the robot
    def update_pose(self, pose):
        self.current_pose = pose

    def naive_path_finder(self, epsilon):
        next_node = self.get_next_node()
        nodes = []
        distances = []
        while(all(distances)):
            nodes.append(next_node)
            distances.append(True)
            next_node = self.get_next_node(next_node)
            if self.path[next_node[0]][next_node[1]] < self.bound:
                break
            elif (next_node[0] - self.current_pose[0] == 0):
                for i in range(len(nodes)):
                    distances[i] = abs(nodes[i][0] - next_node[0]) < epsilon
            else:
                slope = (next_node[1] - self.current_pose[1])/(next_node[0] - self.current_pose[0])
                offset = (next_node[1] - (slope * next_node[0]))
                for i in range(len(nodes)):
                    d = lambda x : math.hypot(nodes[i][0]-x,nodes[i][1] - (slope * x) - offset)
                    z = (nodes[i][0] + (slope * (nodes[i][1] - offset)))/(1 + pow(slope,2))
                    distances[i] = d(z) < epsilon
        return self.get_translation_vector(nodes[-1])

    def find_nearest_non_negative(self, node=None):
        if node is None:
            node = self.current_pose

        checked = [node]
        unchecked = self.get_adjacent(node)

        while len(unchecked) != 0:
            tempUnchecked = []
            for i in unchecked:
                for j in self.get_adjacent(i):
                    if self.path[j[0]][j[1]] == -1 and not (j in checked or j in tempUnchecked):
                        tempUnchecked.append(j)
                    elif self.path[j[0]][j[1]] != -1:
                        return j
                checked.append(i)
            unchecked = tempUnchecked
        return (0,0)
        

    def get_next_node(self, node=None):
        if node is None:
            node = self.current_pose
    
        if self.algorithm == "dijstra":
            adjacent = self.get_adjacent(node)
            min = 0
            for i in range(len(adjacent)):
                if (self.path[adjacent[min][0]][adjacent[min][1]] == -1) or ((self.path[adjacent[i][0]][adjacent[i][1]] != -1) and (self.path[adjacent[i][0]][adjacent[i][1]] < self.path[adjacent[min][0]][adjacent[min][1]])):
                    min = i

            return adjacent[min]
        elif self.algorithm == "a_star":
            return None
        else:
            return None       

    def get_full_path(self, node=None):
        if node is None:
            node = self.current_pose

        path = [node]
        while (self.path[current_pose[0]][current_pose[1]] != 0):
            current_pose = self.get_next_node(current_pose)
            path.append(node)

        return path

    def compute_path(self):
        if self.algorithm == "dijstra":
            self.computer_path_dijstra()
        elif self.algorithm == "a_star":
            self.compute_path_a_star()
        else:
            return

    #returns the vector that coresponds to the direction the robot should move in world coordinates.
    def get_translation_vector(self, node=None):
        next_node = node
        if node is None:
            next_node = self.get_next_node()

        if self.path[self.current_pose[0]][self.current_pose[1]] == -1 and self.path[next_node[0]][next_node[1]] == -1:
            next_node = self.find_nearest_non_negative(self.current_pose) 

        translation = (-self.current_pose[0] + next_node[0], - self.current_pose[1] + next_node[1])
        if translation[0] == 0 and translation[1] == 0:
            return (0,0)
        else:
            hy = math.hypot(translation[0],translation[1])
            return (translation[0]/hy,translation[1]/hy)
    
    def get_adjacent(self, node):
        borders = [(-1,0),(0,1),(1,0),(0,-1)]
        adjacent_nodes = []
        for i in borders:
            if (node[0] + i[0] > -1) and (node[0] + i[0] < len(self.map)) and (node[1] + i[1] > -1) and (node[1] + i[1] < len(self.map[0])):
                adjacent_nodes.append((node[0] + i[0],node[1] + i[1]))
        return adjacent_nodes

    def compute_path_a_star(self):
        self.algorithm = "dijstra"
        self.compute_path()
        temp_path = [[-1]*len(self.map[0]) for i in range(len(self.map))]
        current_pose = self.current_pose

        while (self.path[current_pose[0]][current_pose[1]] != 0):
            temp_path[current_pose[0]][current_pose[1]] = self.path[current_pose[0]][current_pose[1]]
            current_pose = self.get_next_node(current_pose)

        self.path = temp_path
        self.algorithm = "a_star"

    

    def computer_path_dijstra(self):
        self.algorithm = "dijstra"
        checked = [self.destination]
        unchecked = self.get_adjacent(self.destination)
        self.path = [[-1]*len(self.map[0]) for i in range(len(self.map))]
        self.path[self.destination[0]][self.destination[1]] = 0
        for i in unchecked:
             self.path[i[0]][i[1]] = 1
        while len(unchecked) != 0:
            tempUnchecked = []
            for i in unchecked:
                for j in self.get_adjacent(i):
                    if self.path[j[0]][j[1]] == -1 and self.map[j[0]][j[1]] == 1:
                        tempUnchecked.append(j)
                        self.path[j[0]][j[1]] = self.path[i[0]][i[1]] + 1
                checked.append(i)
            unchecked = tempUnchecked
    
    def at_destination(self):
        return self.path[self.current_pose[0]][self.current_pose[1]] < self.bound and self.path[self.current_pose[0]][self.current_pose[1]] != -1
     


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w


def demo_visualize_closestMap(closestMap):
    '''
    Demo: plots the closestMap as a heat map in matplotlib and exits.
    '''
    import matplotlib.pyplot as plt
    plt.imshow(closestMap, cmap='hot', interpolation='nearest')
    plt.show()
    exit(0)
        
    
import sympy as s
t_var = s.Symbol("t")
# x_func = s.cos(t_var)
# y_func = s.sin(t_var) + 1
x_func = 0.2 * t_var * s.cos(t_var)
y_func = 0.2 * t_var * s.sin(t_var)
x_func = s.Piecewise((0.20 * t_var + 0.8, t_var <= 1), (0.45 * t_var + 0.55, t_var <= 2), (-0.12 * t_var + 1.69, t_var <= 3), (1.33, True))
y_func = s.Piecewise((1.15 * t_var + 0.3, t_var <= 1), (1.45, t_var <= 2), (-1.3 * t_var + 4.05, t_var <= 3), (0.15, True))

# start: 0.8, 0.3, 0
# 0.25, 1.15, 1
# 0.45, 0, 1
# -0.12, -1.3, 1
x_lam = s.lambdify(t_var, x_func, "numpy")
y_lam = s.lambdify(t_var, y_func, "numpy")
xp_func = s.diff(x_func, t_var)
yp_func = s.diff(y_func, t_var)
xp_lam = s.lambdify(t_var, xp_func, "numpy")
yp_lam = s.lambdify(t_var, yp_func, "numpy")
a_var = s.Symbol("a")
b_var = s.Symbol("b")
d_func = (x_func - a_var) ** 2 + (y_func - b_var) ** 2
# d = (x - 1) ** 2 + (f - 2) ** 2
dp_func = s.diff(d_func, t_var)
dpp_func = s.diff(dp_func, t_var)
    
def visualize_curve(self):    
    ts = np.arange(0, 100, 0.1)
    particle_cloud_pose_array = PoseArray()
    particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
    for t in ts:
        curr_pose = Pose()
        
        init_pose(curr_pose, x_lam(t), y_lam(t), np.arctan2(yp_lam(t), xp_lam(t)))
        particle_cloud_pose_array.poses.append(curr_pose)
    self.particles_pub.publish(particle_cloud_pose_array)    
    
    

class ParticleFilter:


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
        self.closestMap = np.ascontiguousarray(np.load("computeMap.npy"))
        self.aStarPathMap = np.vectorize(lambda x: (1 if x > 0.155 and x < 0.6 else 0))(self.closestMap)
        
        self.pathFinder = PathFinding(self.aStarPathMap,(274,248),(218,216) ,algorithm="a_star")
        self.pathFinder.compute_path()
        print("THEREs")
        # our addition:
        if (enable_closestMap_viz_demo):
            # demo_visualize_closestMap(self.closestMap) # input the whole cloestMap
            demo_visualize_closestMap(np.array(self.pathFinder.map))
            # demo_visualize_closestMap(np.array(self.pathFinder.path))

        # the number of particles used in the particle filter
        self.find_num_particles = 10**5 # num_particles for searching/convergence
        self.num_particles = 10**3 # num_particles for tracking
        assert (self.find_num_particles >= self.num_particles)
        

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

        
        # buffers used during find behavior
        self.find_poses = np.zeros(shape=(2, self.find_num_particles)) # x, y of poses only
        self.find_yaws = np.zeros(shape=self.find_num_particles) # yaws only
        self.find_weights = np.ones(shape=self.find_num_particles)
        self.finding = True
        self.finding_lock = threading.Lock()
        self.first_pf_cycle = True # do first pf cycle regardless of whether the bot moves


        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.01        
        self.ang_mvmt_threshold = (np.pi / 60)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        self.sound_pub = rospy.Publisher("/sound", Sound, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # initialize in-bound indices
        self.ib_indices = None
        self.init_ib_indices()

        self.robot_estimate = Pose()
        self.robot_estimate_set = False
        self.robot_estimate_updated = False
        self.robot_estimate_cv = threading.Condition()

        # the motion handler
        self.motion = Motion(approach="parametric", init_info=(t_var, x_func, y_func))
        
        # initialize shutdown callback
        rospy.on_shutdown(lambda : self.halt())

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True
        self.first_init = False



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
            
    def init_ib_indices(self):
        ''' 
        Initialize array of indices that are:
        1) in the maze
        2) unblocked
        '''
        while (not self.map_set):
            # wait until the map is set, you idiot
            time.sleep(0.1)
        
        occup_arr = np.array(self.map.data)
        w = self.map.info.width
        h = self.map.info.height
        occup_arr = occup_arr.reshape(w, -h).T
        ib = (occup_arr <= 10) & (occup_arr != -1)
        self.ib_indices = np.array(np.where(ib))
        self.num_ib_indices = self.ib_indices.shape[1]
            
    def scatter_particle_cloud(self):
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
        self.find_yaws[:] = np.random.uniform(low=0, high=2 * np.pi, size=self.find_num_particles)
        self.find_weights[:] = np.ones(shape=self.find_num_particles)
        
        coords = self.ib_indices[:, np.random.randint(low=0, high=self.num_ib_indices, size=self.find_num_particles)]
        xs = coords[0]
        ys = coords[1]
        
        # convert (x,y) to map coordinates
        adj_xs = xs * map_res + map_origin_x
        adj_ys = ys * map_res + map_origin_y
        
        # set particle internal state arrays at corresponding particle position
        self.find_poses[0] = adj_xs # poses[:, i] = [x_i, y_i]
        self.find_poses[1] = adj_ys


    def initialize_particle_cloud(self):
        '''
        Initialize the particle cloud with randomly generated positions and yaws.
        
        Updates are propagated to BOTH particle_cloud and internal state arrays.
        '''
        
        assert (self.map_set) # should be set during or before self.init_ib_indices
            
        self.scatter_particle_cloud()
        
        self.poses[0, :] = self.find_poses[0, :self.num_particles]
        self.poses[1, :] = self.find_poses[1, :self.num_particles]
        self.yaws[:] = self.find_yaws[:self.num_particles] 
        self.weights[:] = self.find_weights[:self.num_particles]
        
        # apply internal states to particle_cloud
        self.update_particle_cloud()
        


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
        if self.finding:
            weights = self.find_weights
            num_particles = self.find_num_particles
        else:
            weights = self.weights
            num_particles = self.num_particles
        
        weights /= np.sum(weights)
        
        
        
        '''
        Float error processing:
        Due to the large number of weights involved, float error is a concern. 
        
        If the normalized weights do not add up to 1, invoking np.random.choice 
        on these weights will fail.
        '''
        sum_weight = np.sum(weights)
        float_error = 1 - sum_weight
        if float_error >= 0:
            # sum_weight <= 1 : must increase sum_weight to 1
            
            # simply add float_error to one of the weights
            weights[0] += float_error
            return
        
        for i in range(num_particles):
            # sum_weight > 1 : must decrease sum_weight to 1
            
            if float_error < -weights[i]:
                '''
                Adding float_error to this weight will make the weight negative,
                which is invalid.
                
                Add only as much of the float error we can (zeroing the weight)
                and move on to the next weight.
                '''
                
                weights[i] = 0
                float_error += weights[i]
                continue
            else:
                '''
                Adding float_error to this weight will make it non-negative,
                which is valid.
                
                Add the float_error to the weight and exit.
                '''
                
                weights[i] += float_error
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
        
    
    def reinitialize(self):
        with self.finding_lock:
            self.finding = True
            self.first_pf_cycle = True
            with self.robot_estimate_cv:
                self.robot_estimate_set = False
                self.robot_estimate_updated = False



    def resample_particles(self):
        '''
        Resample particles in accordance with weights.
        
        WARNING: only updates internal state arrays, NOT particle_cloud
        '''
        
        from collections import Counter
        
        if self.finding:
            num_particles = self.find_num_particles
            poses = self.find_poses
            yaws = self.find_yaws
            weights = self.find_weights
        else:
            num_particles = self.num_particles
            poses = self.poses
            yaws = self.yaws
            weights = self.weights
        
        # sample particle (indices) in accordance with weights
        sample = np.random.choice(np.arange(num_particles), size=num_particles, p=weights)
        
        '''
        Count sampling frequencies
        counts[i] = number of times particle i has been included in the new sample
        '''
        counts = Counter(sample)
        
        dst = 0 # current destination index to fill in resample buffers
        i = 0 # particle index in sample
        end_reached = False
        for i in counts.keys():
            n = counts[i]
            
            if dst + n >= self.num_particles:
                end_reached =  True
                n = self.num_particles - dst
            
            # fill n-long stretch in the buffer with the state of particle i
            # this simulates picking n copies of particle i
            self.poses_resample[:,dst:dst+n] = poses[:, i][:, np.newaxis]
            self.yaws_resample[dst:dst+n] = yaws[i]
            self.weights_resample[dst:dst+n] = weights[i]
            
            # advance to the next available stretch in the buffer
            dst += n
        
            if end_reached:
                break
        
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
                self.first_pf_cycle or 
                True):
                self.first_pf_cycle = False

                # This is where the main logic of the particle filter is carried out
                with self.finding_lock:
                    print("START ====")
                    
                    prevT = time.time()
                    if self.finding:                
                        self.scatter_particle_cloud()
                    else:
                        self.update_particles_with_motion_model()
                    currT = time.time()
                    print("motion", currT - prevT)
                    
                    prevT = currT
                    self.update_particle_weights_with_measurement_model(data)
                    currT = time.time()
                    print("measure", currT - prevT)
                    
                    prevT = currT
                    self.normalize_particles()
                    currT = time.time()
                    print("norm", currT - prevT)

                    prevT = currT
                    self.resample_particles()
                    currT = time.time()
                    print("resample", currT - prevT)
                    
                    # apply internal states to particle buffer
                    self.update_particle_cloud()

                    prevT = currT
                    self.update_estimated_robot_pose()
                    currT = time.time()
                    print("update pose", currT - prevT)

                    prevT = currT
                    self.publish_particle_cloud()
                    currT = time.time()
                    print("pub cloud", currT - prevT)

                    prevT = currT
                    self.publish_estimated_robot_pose()
                    currT = time.time()
                    print("pub pose", currT - prevT)
                    print("END ====\n")                
                    
                    self.finding = False






                if self.pathFinder.at_destination():
                    self.sound_pub.publish(Sound(0))
                    rospy.sleep(4)
                    rospy.signal_shutdown("got bored")            

                # start = time.time()
                # self.motion.move(self.robot_estimate)
                # print("move:", time.time()-start)         

                self.odom_pose_last_motion_update = self.odom_pose
                self.first_init = False



    def update_estimated_robot_pose(self):
        ''' Update the estimated robot pose. '''
        
        # Compute average position and yaw (weighted with particle weights).
        av_x = np.average(self.poses[0, :], weights=self.weights)
        av_y = np.average(self.poses[1, :], weights=self.weights)
        av_yaw = np.average(self.yaws, weights=self.weights)
        
        # Update current robot pose to computed averages
        pose = self.robot_estimate
        # pose.position = Point()
        with self.robot_estimate_cv:
            pose.position.x = av_x
            pose.position.y = av_y
            pose.position.z = 0
            # Need to convert yaw to quaternion
            quat = quaternion_from_euler(0,0,av_yaw)
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            
            self.robot_estimate_set = True
            self.robot_estimate_updated = True
            self.robot_estimate_cv.notify_all()
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
        angle_incr = 3 # 3 increment ~ 0.9 degrees
        is_rplidar = False
        
        if is_rplidar:
            angle_indices = np.arange(0, 1147, angle_incr)
            num_angles = len(angle_indices)
            
            # convert from lidar angle indices to angles
            lidar_angles = angle_indices * (2 * np.pi) / 1147 - np.pi
        else:
            angle_indices = np.arange(0, 360, angle_incr)
            num_angles = len(angle_indices)
            
            # convert from lidar angle indices to angles
            lidar_angles = angle_indices * (2 * np.pi) / 360
        
        
        # angle_incr = 1 if self.first_init else 90 # 3 increment ~ 0.9 degrees
        # angle_indices = np.arange(0, 360, angle_incr)
        # num_angles = len(angle_indices)
        
        # # convert from lidar angle indices to angles
        # lidar_angles = angle_indices * (2 * np.pi) / 360
        
        
        
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
        
        if self.finding:
            num_particles = self.find_num_particles
            yaws = self.find_yaws
            poses = self.find_poses
            weights = self.find_weights
        else:
            num_particles = self.num_particles
            yaws = self.yaws
            poses = self.poses
            weights = self.weights
        
        # reorient "ranges" vectors to particle yaw...
        trans_dxs = ranges * np.cos(lidar_angles + yaws[:, None]) + poses[0][:, None]
        trans_dys = ranges * np.sin(lidar_angles + yaws[:, None]) + poses[1][:, None]
        
        norm_xs = ((trans_dxs - pos_x)/map_res).astype(np.int64, copy=False)
        norm_ys = ((trans_dys - pos_y)/map_res).astype(np.int64, copy=False)
        
        # closest obstacle distances; initialized to out-of-bound penalty
        # (until proven to be "in_bound", assume all "ranges" are out-of-bound)
        dists = np.zeros(shape=(num_particles, num_angles)) + oob_penalty
        
        # boolean mask indicating all in-bound range indices
        in_bound = (norm_xs >= 0) & (norm_xs < width) & (norm_ys >= 0) & (norm_ys < height)
        
        # inititalize only in-bound closest map distances
        dists[in_bound] = self.closestMap[norm_xs[in_bound], norm_ys[in_bound]]
        zr_zm = 0
        if z_max != 0:
            # set zr_zm IFF z_max is nonzero
            zr_zm = z_random/z_max
        
        # compute likelihood probabilities of each distance
        prob_ds = np.exp(-(dists**2)/(2 * (sigma_hit**2))) / (sigma_hit * np.sqrt(2 * np.pi))
        
        # set particle weight with product of probabilities
        weights[:] = np.prod(preserve_factor * z_hit * prob_ds + zr_zm, axis=1)
        return
    
    
    
        
        for i in range(num_particles):
            # yaw of ith particle
            curr_yaw = yaws[i]
            
            # x,y positions of ith particle
            x_i = poses[0, i]
            y_i = poses[1, i]
            
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
            dists[in_bound] = self.closestMap[norm_xs[in_bound], norm_ys[in_bound]]
            assert(len(dists) == num_angles)
            
            
            # compute likelihood probabilities of each distance
            prob_ds = np.exp(-pow(dists,2)/(2 * pow(sigma_hit,2))) / (sigma_hit * np.sqrt(2 * np.pi))
            
            # set particle weight with product of probabilities
            weights[i] = np.prod(preserve_factor * z_hit * prob_ds + zr_zm)


        

    def update_particles_with_motion_model(self):        
        # we should not call motion_model during finding
        assert (not self.finding)
        
        '''
        Updates particles' positions and yaws by the latest movement of the bot.
        
        WARNING: only updates internal state arrays, NOT particle_cloud
        '''
        
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
        
        # standard deviations for noise
        dist = np.hypot(dx_raw, dy_raw)
        angle_increment = 6 * np.pi / 180 * min(abs(dyaw_raw) / 0.75, 1)
        linear_increment = 0.1 * min(dist / 0.225, 1)
        
        # generate random tapes for movement noise
        # e.g. dx_noise[i] == x translation noise for particle i
        dx_noise = gaussian_scalars(linear_increment, self.num_particles)
        dy_noise = gaussian_scalars(linear_increment, self.num_particles)
        dyaw_noise = gaussian_scalars(angle_increment, self.num_particles)
        
        # compute deltas for each particle, adjusted by noise
        dxs = dx_noise + dx_raw
        dys = dy_noise + dy_raw
        dyaws = dyaw_noise + dyaw_raw
        
        # update particle positions by corresponding (transformed) deltas
        thetas = self.yaws - old_yaw
        
        self.poses[0, :] += (dxs * np.cos(thetas) - dys * np.sin(thetas))
        self.poses[1, :] += (dxs * np.sin(thetas) + dys * np.cos(thetas))            
        
        # update particle yaws by corresponding deltas (modulo to wrap around)
        self.yaws = (self.yaws + dyaws) % (2 * np.pi)
        
        # print(f"dx: {dx_raw}, dy: {dy_raw}, dyaw: {dyaw_raw}")
        
        
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # TODO


    def halt(self):
        self.motion.halt()
        with self.robot_estimate_cv:
            self.robot_estimate_set = True
            self.robot_estimate_updated = True
            self.robot_estimate_cv.notify_all()
    

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

if __name__=="__main__":
    

    pf = ParticleFilter()
    print("HERE")
    
    
    
    rospy.spin()
    exit(0)
    # !! PROCEED NO FURTHER !!
    
    
    
    
    
    
    
    
    
    
    
    
    # global robot_estimate_set
    # global self.robot_estimate_updated
    # global self.robot_estimate_cv
    
    curr_t = 0
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.on_shutdown(lambda : halt(pub_cmd_vel))
    
    while not robot_estimate_set:
        print("not set")
        rospy.sleep(1)
    print("set!")
        
    # while not rospy.is_shutdown():
    #     time.sleep(0.5)
    #     pass

    while not rospy.is_shutdown():
        with self.robot_estimate_cv:
            while not self.robot_estimate_updated:
                self.robot_estimate_cv.wait()
            print("updated!")
            curr_x = robot_estimate.position.x
            curr_y = robot_estimate.position.y
            curr_yaw = get_yaw_from_pose(robot_estimate)
            self.robot_estimate_updated = False
        if rospy.is_shutdown():
            break
            
        start = time.time()
        is_approx = True
        
        start=time.time()
        d_curr = d_func.subs(a_var, curr_x).subs(b_var, curr_y)
        dp_curr = dp_func.subs(a_var, curr_x).subs(b_var, curr_y)


        if not is_approx:
            # roots = s.Poly(dp, x).nroots()
            roots = s.real_roots(dp_curr, t_var)
            t = min(roots, key = lambda r : d_curr.subs(t_var, r).evalf())
            clos_x_sym = x_lam(t)
            clos_y_sym = y_lam(t)
            
            clos_x = clos_x_sym.evalf()
            clos_y = clos_y_sym.evalf()
            
            curve_x = xp_func.subs(t_var, clos_x_sym).evalf()
            curve_y = yp_func.subs(t_var, clos_x_sym).evalf()
            
        else:
            from scipy.optimize import minimize_scalar
            d_curr_lam = s.lambdify(t_var, d_curr, "numpy")
            
            # 0.01 gives a sense of urgency; always pick at least slightly further along the curve
            result = minimize_scalar(d_curr_lam, bounds=(curr_t + 0.01, curr_t + 1), method='bounded')
            t = result.x
        
            clos_x = x_lam(t)
            clos_y = y_lam(t)
        
            curve_x = xp_lam(t)
            curve_y = yp_lam(t)
            
            curr_t = t
        
        corr_weight = np.e ** (100 * np.hypot(clos_x - curr_x, clos_y - curr_y)) - 1
        corr_vel = np.array([clos_x - curr_x, clos_y - curr_y]) * 16
        curve_vel = np.array([curve_x, curve_y]) * 8
        total_vel = (corr_vel + curve_vel).astype(float)
        # np.dot()
        target_yaw = np.arctan2(total_vel[1], total_vel[0])
        ang_error = wrapto_pi(target_yaw - curr_yaw)
        pos_error = np.hypot(float(clos_x - curr_x), float(clos_y - curr_y))
        print(f"pos_error (m): {pos_error}, ang_error (rad): {ang_error}")

        
        # print("time: ", time.time() - start)
        
        # lin_error = absolute_cutoff(dist + 0.4, limit=1) * (abs(ang_error) / np.pi - 1) ** 12
        lin_error = 0.4 * (abs(ang_error) / np.pi - 1) ** 16
        pub_cmd_vel.publish(Twist(linear=Vector3(0.5*lin_error,0,0),angular=Vector3(0,0,0.5*ang_error)))
        print("MOVE: ", time.time()-start)
    rospy.spin()









