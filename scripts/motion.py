import rospy
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
import numpy as np
import sympy as s


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



class Motion:
    def __init__(self, approach, init_info):    
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.on_shutdown(lambda : halt(self.pub_cmd_vel))
        
        if approach == "naive":
            
            return
            
        if approach == "parametric":
            self.move = self.move_parametric
            
            t_var, x_func, y_func = init_info
            print("t_var", t_var)
            print("x_func", x_func)
            print("_func", y_func)
            self.t_var = t_var
            self.x_func = x_func
            self.y_func = y_func
            
            self.x_lam = s.lambdify(t_var, x_func, "numpy")
            self.y_lam = s.lambdify(t_var, y_func, "numpy")
            self.xp_func = s.diff(x_func, t_var)
            self.yp_func = s.diff(y_func, t_var)
            self.xp_lam = s.lambdify(t_var, self.xp_func, "numpy")
            self.yp_lam = s.lambdify(t_var, self.yp_func, "numpy")
            self.a_var = s.Symbol("a")
            self.b_var = s.Symbol("b")
            self.d_func = (x_func - self.a_var) ** 2 + (y_func - self.b_var) ** 2
            # d = (x - 1) ** 2 + (f - 2) ** 2
            self.dp_func = s.diff(self.d_func, t_var)
            self.dpp_func = s.diff(self.dp_func, t_var)
            
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
            
        is_approx = True
        
        t_var = self.t_var
        x_lam = self.x_lam
        y_lam = self.y_lam
        xp_func = self.yp_func
        yp_func = self.yp_func
        xp_lam = self.xp_lam
        yp_lam = self.yp_lam
        a_var = self.a_var
        b_var = self.b_var
        d_func = self.d_func
        dp_func = self.dp_func
        curr_t = self.curr_t
        
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
            print("t: ", t)
        
            clos_x = x_lam(t)
            clos_y = y_lam(t)
        
            curve_x = xp_lam(t)
            curve_y = yp_lam(t)
            
            self.curr_t = t
            
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
            lin_error = (abs(ang_error) / np.pi - 1) ** 16
        
        self.pub_cmd_vel.publish(Twist(linear=Vector3(lin_error,0,0),angular=Vector3(0,0,ang_error)))
        

    