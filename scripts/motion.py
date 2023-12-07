import rospy
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
import numpy as np
import sympy as s

from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Header

import scipy
from scipy.interpolate import splprep, splev

def to_rviz_coords(self, ind_x, ind_y):
    # map_res = self.map.info.resolution
    # pos_x = self.map.info.origin.position.x
    # pos_y = self.map.info.origin.position.y
    
    x = (ind_x * self.map_res) + self.pos_x
    y = (ind_y * self.map_res) + self.pos_y
    return x, y

def to_closestMap_indices(self, x, y):
    ind_x = int(((x - self.pos_x)/self.map_res))
    ind_y = int(((y - self.pos_y)/self.map_res))
    return ind_x, ind_y


def halt(publisher):
    '''Halts the robot. i.e. publishes a 0 Twist to cmd_vel'''
    publisher.publish(Twist(linear=Vector3(0,0,0),angular=Vector3(0,0,0)))
    
def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

def wrapto_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi



# enable naive debug to see the next node and next_yaw
enable_naive_debug = True

def init_pose(pose: Pose, x, y, yaw):
    # pose.position = Point()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    # Need to convert yaw to quaternion
    quat = quaternion_from_euler(0,0, yaw)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]




class Motion:
    def publish_pose(self, pose: Pose):
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header = Header(stamp=rospy.Time.now(), frame_id="map")
        self.robot_estimate_pub.publish(pose_stamped)
        
    def publish_curve(self):
    
        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id="map")
        particle_cloud_pose_array.poses = self.curve_poses
    
        self.particles_pub.publish(particle_cloud_pose_array)
        

    def __init__(self, approach, pathFinder, closestMap):  
        self.pathFinder =  pathFinder
        self.map_res = closestMap.info.resolution
        self.pos_x = closestMap.info.origin.position.x
        self.pos_y = closestMap.info.origin.position.y
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)


        if enable_naive_debug:
            self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)
        
        if approach == "naive":
            self.move = self.move_naive
            self.curve_poses = []
            
            return
            
        if approach == "parametric":
            self.move = self.move_parametric
            self.path_set = False
            self.tck = None
            self.curr_t = 0
            self.curve_poses = np.array([Pose() for i in range(int(1/0.0001))]) 
            return
            
    
    def halt(self):
        '''Halts the robot. i.e. publishes a 0 Twist to cmd_vel'''
        self.pub_cmd_vel.publish(Twist(linear=Vector3(0,0,0),angular=Vector3(0,0,0)))
            
        
    
    def move_parametric(self, curr_pose):
        curr_x = curr_pose.position.x
        curr_y = curr_pose.position.y
        curr_yaw = get_yaw_from_pose(curr_pose)
        curr_pose_updated = False

        indc_x, indc_y = to_closestMap_indices(self, curr_x, curr_y)
        #if self.pathFinder.shortest_dists[indc_x][indc_y] == -1:
        #    self.move_naive() 
        #    self.curve_poses = np.array([Pose() for i in range(int(1/0.0001))]) 
        
        from scipy.interpolate import splprep, splev
        if (not self.path_set):
            print("pre sel", self.pathFinder.current_pose)
            
            self.pathFinder.update_pose((indc_x, indc_y))
            print("pos sel", self.pathFinder.current_pose)
            
            self.pathFinder.compute_path()
            # self.pathFinder.reduce_path(1) # actually works better without this
            
            pathxs = self.pathFinder.path[:, 0] * self.map_res + self.pos_x
            pathys = self.pathFinder.path[:, 1] * self.map_res + self.pos_y
            
            tck, u = splprep([pathxs, pathys], k=3,s=0.007)
            self.tck = tck
            
            def init_curve():    
                ts = np.arange(0, 1, 0.0001)
                x3s, y3s = splev(ts, tck)
                
                print(x3s)
                xp3s, yp3s = splev(ts, tck, der=1)
                particle_cloud_pose_array = PoseArray()
                particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id="map")
                
                for i in range(0, len(ts), 100):
                    curr_pose = self.curve_poses[i]

                    init_pose(curr_pose, x3s[i], y3s[i], np.arctan2(yp3s[i], xp3s[i]))
            
            init_curve()
            self.path_set = True
        self.publish_curve()
        
        
        
        
        # from scipy.interpolate import splprep, splev
        # if (not self.path_set):
        #     print("pre sel", self.pathFinder.current_pose)
        #     indc_x, indc_y = to_closestMap_indices(self, curr_x, curr_y)
        #     self.pathFinder.update_pose((indc_x, indc_y))
        #     print("pos sel", self.pathFinder.current_pose)
            
        #     self.pathFinder.compute_path()
        #     self.pathFinder.reduce_path(1) # actually works better without this
            
        #     pathxs = self.pathFinder.path[:, 0] * self.map_res + self.pos_x
        #     pathys = self.pathFinder.path[:, 1] * self.map_res + self.pos_y
            
        #     tck, u = splprep([pathxs, pathys], k=3,s=0.007)
        #     self.tck = tck
            
        #     def init_curve():    
        #         ts = np.arange(0, 1, 0.0001)
        #         x3s, y3s = splev(ts, tck)
                
        #         print(x3s)
        #         xp3s, yp3s = splev(ts, tck, der=1)
        #         particle_cloud_pose_array = PoseArray()
        #         particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id="map")
                
        #         for i in range(0, len(ts), 100):
        #             curr_pose = self.curve_poses[i]

        #             init_pose(curr_pose, x3s[i], y3s[i], np.arctan2(yp3s[i], xp3s[i]))
            
        #     init_curve()
        #     self.path_set = True
        # self.publish_curve()
        
        
        
        
        # start=time.time()
        
        def distance_to_curve(point, tck):
            x, y = point
            
            def objective(t):
                curve_point = splev(t, self.tck)
                dist = np.sqrt((x - curve_point[0])**2 + (y - curve_point[1])**2)
                return dist

            result = scipy.optimize.minimize_scalar(objective, bounds=(self.curr_t+0.001, self.curr_t + 0.01), method="bounded")
            return result.x

            
        tck = self.tck
        t = distance_to_curve((curr_x, curr_y), tck)
        self.curr_t = t
        
        closest_point_on_curve = splev(t, tck)
        derivative_of_closest_point = splev(t, tck, der=1)
        
        clos_x = closest_point_on_curve[0]
        clos_y = closest_point_on_curve[1]
        curve_x = derivative_of_closest_point[0]
        curve_y = derivative_of_closest_point[1]
        
        
        k = len(self.pathFinder.path) / 924
        corr_weight = 30
        corr_vel = np.array([clos_x - curr_x, clos_y - curr_y]) * corr_weight
        curve_vel = np.array([curve_x, curve_y])
        total_vel = (corr_vel + curve_vel).astype(float)
        
        if enable_naive_debug:
            next_pose = Pose()
            init_pose(next_pose, clos_x, clos_y, np.arctan2(total_vel[1], total_vel[0]))
            # self.publish_pose(next_pose)

        target_yaw = np.arctan2(total_vel[1], total_vel[0])
        ang_error = wrapto_pi(target_yaw - curr_yaw)
        pos_error = np.hypot(float(clos_x - curr_x), float(clos_y - curr_y))
        print(f"pos_error (m): {pos_error}, ang_error (rad): {ang_error}")

        tempx = int((curr_pose.position.x - self.pos_x)/self.map_res)
        tempy = int((curr_pose.position.y - self.pos_y)/self.map_res)


        self.pathFinder.update_pose((tempx,tempy))
            
        # print("time: ", time.time() - start)
        lin_error = (abs(ang_error) / np.pi - 1) ** 10
        
        # this combo is also good:
        # self.pub_cmd_vel.publish(Twist(linear=Vector3(0.8*lin_error,0,0),angular=Vector3(0,0,2*ang_error)))
        self.pub_cmd_vel.publish(Twist(linear=Vector3(4*k*lin_error,0,0),angular=Vector3(0,0,2*ang_error)))
        

    def move_naive(self,curr_pose):
       # This is where the main logic of the particle filter is carried out

        
        tempx = int((curr_pose.position.x - self.pos_x)/self.map_res)
        tempy = int((curr_pose.position.y - self.pos_y)/self.map_res)


        self.pathFinder.update_pose((tempx,tempy))

        print("distance:",self.pathFinder.shortest_dists[tempx][tempy])
                
        move_vector, next_node = self.pathFinder.naive_path_finder(0.05/self.map_res)
                

        # next_node = ((next_node[0] * map_res), (next_node[1] * map_res))
        next_yaw = np.arctan2(move_vector[1],move_vector[0])
        mv_x = np.cos(next_yaw)
        mv_y = np.sin(next_yaw)
        
        print("HERE")
        self.pathFinder.compute_path()
        self.curve_poses = np.array([Pose() for i in range(int(len(self.pathFinder.path)/10)+1)])
        for i in range(len(self.pathFinder.path)-1):
            if i % 10 == 0:
                m = self.pathFinder.path[i]
                n = self.pathFinder.path[i+1]
                temp_yaw = np.arctan2(n[1] - m[1], n[0] - m[0])
                newPose = Pose()
                init_pose(newPose, (m[0] * self.map_res) + self.pos_x , (m[1] * self.map_res) + self.pos_y, temp_yaw)
                self.curve_poses[int(i/10)] = newPose
        self.publish_curve()
        print("HERE")

        error = 0.25
        print(next_yaw,get_yaw_from_pose(curr_pose))
        ang_vel = 10 * wrapto_pi(next_yaw- get_yaw_from_pose(curr_pose))
        #np.sign(np.sin(next_yaw - get_yaw_from_pose(curr_pose))) * np.arccos(mv_y * move_vector[0] + mv_x * move_vector[1])/np.pi

        lin_vel =  200 * self.map_res * pow((1 + np.cos(ang_vel))/2,10)

        if(lin_vel < 0.00001):
            lin_vel = 0
        if(abs(ang_vel) < 0.00001):
            ang_vel = 0

        print(lin_vel,ang_vel)
        
        self.pub_cmd_vel.publish(Twist(linear=Vector3(error * lin_vel,0,0),angular=Vector3(0,0,error * ang_vel)))    
        
        return
    