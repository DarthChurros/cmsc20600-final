from multiprocessing import Array, Lock, Value, Condition

import numpy as np
from numpy.typing import ArrayLike
import ctypes as c

# Configs
MAX_PATH_LEN = 500

class SharedInterface():
    def __init__(self):
        '''
        Notes: 
        - value attributes must be...
            -> read as: new_var = shared_val.value 
                     OR new_var = shared_val <DO NOT DO THIS; THIS IS MUTABLE MEMORY>
            -> writ as: shared_val.value = new_value
        - private (denoted with "__" prefix) attributes should be get/set
        with their corresponding methods
        '''
        
        self.finding_lock = Lock()
        self.finding = Value('i', True, lock=False)
        self.first_pf_cycle = Value('i', True, lock=False) # do first pf cycle regardless of whether the bot moves
        
        self.robot_estimate_cv = Condition()
        self.__robot_estimate = np.frombuffer(Array('d', [0] * 3, lock=False)) # update using helper
        self.robot_estimate_initialized = Value('i', False, lock=False)
        self.robot_estimate_updated = Value('i', False, lock=False)
        
        self.path_cv = Condition()
        self.path_initialized = Value('i', False, lock=False)
        self.__path_xs = np.frombuffer(Array(c.c_double, [0] * MAX_PATH_LEN, lock=False))
        self.__path_ys = np.frombuffer(Array(c.c_double, [1] * MAX_PATH_LEN, lock=False))
        
        # self.closestMap = np.ascontiguousarray(np.load("computeMap.npy"))
        # np.frombuffer(mp_arr.get_obj())
        self.closestMap = np.frombuffer(Array(c.c_double, [0] * MAX_PATH_LEN, lock=False))
        
        '''
        (Aside)
        Values can also be struct/union types! See:
        - https://docs.python.org/3/library/multiprocessing.html#multiprocessing.Value
        - https://docs.python.org/3/library/ctypes.html#structures-and-unions 
        '''
        
    
    def read_robot_estimate(self):
        '''Retrieves local copies of robot_estimate values'''
        x, y, yaw = self.__robot_estimate
        return x, y, yaw
        
    def writ_robot_estimate(self, x, y, yaw):
        '''Updates the shared robot_estimate'''
        self.__robot_estimate[:] = x, y, yaw
        
    def read_path(self):
        '''Creates a deep copy of the shared path array'''
        return np.copy([self.__path_xs, self.__path_ys])
    def read_path_shallow(self):
        # https://stackoverflow.com/questions/9754034/can-i-create-a-shared-multiarray-or-lists-of-lists-object-in-python-for-multipro
        '''Creates a shallow copy of the shared path array'''
        # return np.frombuffer(self.__path_xs)
        return [self.__path_xs, self.__path_ys]
        
    def writ_path(self, src: ArrayLike):
        '''
        Writes the contents of given src array (which MUST be the same
        shape as the path array) to the shared path array.
        '''
        assert(src.shape == (2, MAX_PATH_LEN))
        self.__path_xs[:] = src[0]
        self.__path_ys[:] = src[1]