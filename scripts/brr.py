# brr.py
import rospy
import threading
from shm import SharedInterface
import numpy as np
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
        print("Hi from brr")
        from move import SharedInterface
        
        import time
        with self.shm.finding_lock:
            print("yos")
            self.shm.finding.value = 25
        
        
        with self.shm.finding_lock:
            print("0acq lock")
        
            # finding = bool(self.shm.finding)
            # print("first", finding)
            # finding = False
            # print("second", finding)
            

            arr = np.array([np.arange(500), [0] *500])
            self.shm.writ_path(arr)
        print("0rel lock")
        
        time.sleep(5)
        
        with self.shm.finding_lock:
            print("1acq lock")
            print("1 received: ", self.shm.read_path())
        
        print("1rel lock")
        
        with self.shm.robot_estimate_cv:
            while not self.shm.robot_estimate_initialized:
                print("not init... waiting...")
                self.shm.robot_estimate_cv.wait()
                print("try wake")
                
            print("woke up!")
            
            print(self.shm.read_robot_estimate())
            
            
        
            
            
        
        
        # self.shm.finding=False
        # time.sleep(4)
        # print("four", self.shm.finding)
        


if __name__=="__main__":
    pass
    