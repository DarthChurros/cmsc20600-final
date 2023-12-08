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

import multiprocessing
import time
if __name__=="__main__":
    shm = SharedInterface()
    
    brr_thread = Process(target=lambda shm : Brr(shm), args=(shm,))
    brr_thread.start()
    
    print("move", shm.finding.value)
    
    import time
    time.sleep(1)
    with shm.finding_lock:
        print(">> 2: mov rcv finding")
        print("mov: finding = ", shm.finding)
        print("<< 2: mov rcv finding")
    
    
    with shm.finding_lock:
        print(">> 3: mov set path")
        arr = np.array([np.arange(500), [0] *500])
        shm.writ_path(arr)
        print("<< 3: mov set path")
    
    
    with shm.robot_estimate_cv:
        while not shm.robot_estimate_initialized:
            shm.robot_estimate_cv.wait()
        
        print(">> 6: mov rcv robot_estimate")
        print("mov: robot_estimate =", shm.read_robot_estimate())
        print("<< 6: mov rcv robot_estimate")
    exit(0)
    