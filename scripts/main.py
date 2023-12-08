# main.py
from particle_filter import ParticleFilter
from motion import Motion
from shm import SharedInterface
from multiprocessing import Pool, Process
import numpy as np
import time as time

# Signalling and Locking
from geometry_msgs.msg import Pose

if __name__=="__main__":
    shm = SharedInterface()
    
    motion = Process(target=lambda shm : Motion(shm), args=(shm,))
    pf = Process(target=lambda shm : ParticleFilter(shm), args=(shm,))
    
    pf.start()
    
    
    time_limit = 10
    start_time = time.time()
    curr_time = start_time
    end_time = start_time + time_limit
    while curr_time < end_time:
        time.sleep(1)
        curr_time = time.time()
    # pf.join()
    