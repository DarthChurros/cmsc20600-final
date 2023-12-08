# brr.py
import rospy
import threading
from shm import SharedInterface
import numpy as np
import time
# from move import SharedInterface

# Signalling and Locking



finding = True
first_pf_cycle = True # do first pf cycle regardless of whether the bot moves
finding_lock = threading.Lock()

class Brr:
    def __init__(self, shm: SharedInterface):
        self.ff = True
        self.shm = shm
        global yar
    def run(self):
        with self.shm.finding_lock:
            print(">> 1: brr set finding")
            self.shm.finding.value = 25
            self.shm.finding.value = False
            print("<< 1: brr set finding")
        
        time.sleep(2)
        
        with self.shm.finding_lock:
            print(">> 4: brr rcv path")
            print("brr: path = ", self.shm.read_path())
            print("<< 4: brr rcv path")
            
            
        time.sleep(2)
        
        with self.shm.robot_estimate_cv:
            print(">> 5: brr set robot_estimate")
            self.shm.writ_robot_estimate(1, 2, 3)
            self.shm.robot_estimate_initialized.value = True
            self.shm.robot_estimate_cv.notify()
            print("<< 5: brr set robot_estimate")
        


if __name__=="__main__":
    pass
    