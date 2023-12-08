# move.py
import rospy
import threading
from particle_filter import ParticleFilter
from brr import Brr
from shm import SharedInterface
from multiprocessing import Pool, Process
import numpy as np

# Signalling and Locking
from geometry_msgs.msg import Pose


        
        
        

class Motion:
    def __init__(self):
        self.ff = True


def start_particle_filter(shm):
    pf = ParticleFilter(shm)
    pf.run()
    
def start_brr(shm):
    pf = Brr(shm)
    pf.run()


import multiprocessing
import time
if __name__=="__main__":
    shm = SharedInterface()
    
    brr_thread = Process(target=start_brr, args=(shm,))
    brr_thread.start()
    
    print("move", shm.finding)
    
    import time
    time.sleep(5)
    with shm.finding_lock:
        print("the finding", shm.finding)
    # exit(0)
    
    
    
    print("waiting on finding_lock...")
    with shm.finding_lock:
        print("Aacq lock")
        
        print("A received: ", shm.read_path())
        shm.writ_path(np.array([[69] * 500, [13] * 500]))
    
    print("Arel lock")
    
    with shm.robot_estimate_cv:
        shm.writ_robot_estimate(1,2,3)
        shm.robot_estimate_initialized.value = True
        print("set true")
        shm.robot_estimate_cv.notify()
        

    
    
    
    # pf_thread = Process(target=start_particle_filter, args=(shm,))
    # pf_thread.daemon = True
    
    
    # pf_thread.start()
    # print("bye!")
    # time.sleep(3)
    # print(str(buf))
    
    
    # print("ou")
    # with shm.finding_cv:
    #     print("ou lock!")
    #     import time
    # print("ou release lock!")


# def node_function(node_name):
#     rospy.init_node(node_name, anonymous=True)
#     rate = rospy.Rate(1)  # Adjust the rate as needed

#     while not rospy.is_shutdown():
#         # Your node logic here
#         rospy.loginfo(f"Hello from {node_name}!")
#         rate.sleep()

# if __name__ == '__main__':
#     # List of node names
#     node_names = ['node1', 'node2', 'node3']
    
#     rospy.init_node("node1", anonymous=True)
#     rospy.init_node("node2", anonymous=True)
#     rospy.init_node("node3", anonymous=True)
#     exit(0)

#     # Create and start a thread for each node
#     threads = []
#     for name in node_names:
#         thread = threading.Thread(target=node_function, args=(name,))
#         threads.append(thread)
#         thread.start()

#     try:
#         # Your main program logic here
#         rospy.loginfo("Main program starting...")
#         rospy.spin()  # This keeps the main thread alive while the node threads run

#     except rospy.ROSInterruptException:
#         pass

#     finally:
#         # Stop all node threads when the main program is interrupted or exits
#         rospy.loginfo("Main program interrupted. Stopping node threads...")
#         for thread in threads:
#             thread.join()
    