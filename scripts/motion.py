import rospy
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
import numpy as np
import sympy as s

from geometry_msgs.msg import Pose, PoseStamped
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
        
    
    
    def __init__(self, approach, pathFinder, pub_cmd_vel, map_res, map_origin_x, map_origin_y):  
        self.pathFinder =  pathFinder
        self.pub_cmd_vel = pub_cmd_vel
        self.map_res = map_res
        self.pos_x = map_origin_x
        self.pos_y = map_origin_y



        if enable_naive_debug:
            self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)
        
        if approach == "naive":
            self.move = self.move_naive
            
            return
            
        if approach == "parametric":
            self.move = self.move_parametric
            from scipy.interpolate import splprep, splev
        
            pathxs = self.pathFinder.path[:, 0] * map_res + self.pos_x
            pathys = self.pathFinder.path[:, 1] * map_res + self.pos_y
            tck, u = splprep([pathxs, pathys], k=3,s=0.01)
            # tck, u = splprep([pathxs, pathys], k=1,s=0)
            init_info=(tck,)
            tck = init_info[0]
            self.tck = tck
            
            self.curr_t = 0
            return
            
    
    def halt(self):
        '''Halts the robot. i.e. publishes a 0 Twist to cmd_vel'''
        self.pub_cmd_vel.publish(Twist(linear=Vector3(0,0,0),angular=Vector3(0,0,0)))
            
        
    
    def move_parametric(self, curr_pose):
        curr_x = curr_pose.position.x
        curr_y = curr_pose.position.y
        curr_yaw = get_yaw_from_pose(curr_pose)
        curr_pose_updated = False
        
        # start=time.time()
        
        def distance_to_curve(point, tck):
            x, y = point
            
            def objective(t):
                curve_point = splev(t, self.tck)
                dist = np.sqrt((x - curve_point[0])**2 + (y - curve_point[1])**2)
                return dist

            result = scipy.optimize.minimize_scalar(objective, bounds=(self.curr_t, self.curr_t + 0.01), method="bounded")
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
        
        
        corr_weight = 40
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

            
        # print("time: ", time.time() - start)
        lin_error = (abs(ang_error) / np.pi - 1) ** 12
        
        # this combo is also good:
        # self.pub_cmd_vel.publish(Twist(linear=Vector3(0.8*lin_error,0,0),angular=Vector3(0,0,2*ang_error)))
        self.pub_cmd_vel.publish(Twist(linear=Vector3(lin_error,0,0),angular=Vector3(0,0,2*ang_error)))
        

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
        
        #if enable_naive_debug:
            #next_pose = Pose()
            #print(move_vector)
            #nextx, nexty = to_rviz_coords(pf, next_node[0], next_node[1])
            #init_pose(next_pose, nextx, nexty, next_yaw)
            #self.publish_pose(next_pose)

        error = 0.25
        print(next_yaw,get_yaw_from_pose(curr_pose))
        ang_vel = np.sign(np.sin(next_yaw - get_yaw_from_pose(curr_pose))) * (mv_x * move_vector[0] + mv_y * move_vector[1])

        lin_vel =  80 * self.map_res * pow((1 + np.cos(ang_vel))/2,20)

        if(lin_vel < 0.00001):
            lin_vel = 0
        if(abs(ang_vel) < 0.00001):
            ang_vel = 0

        print(lin_vel,ang_vel)
        
        self.pub_cmd_vel.publish(Twist(linear=Vector3(error * lin_vel,0,0),angular=Vector3(0,0,error * ang_vel)))    
        
        return
    