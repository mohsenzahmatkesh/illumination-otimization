#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 17 12:08:35 2025

@author: Mohsen Zahmatkesh
"""
import numpy as np
from singlepoint_illumination import singlepoint_optimization
import rospy
from geometry_msgs.msg import Pose
np.float = float 
np.int = int     

class Experiment_sim():
    
    def __init__(self):
        
        rospy.init_node("illumination_command_generation")
        
        self.optimized_illumination_results = singlepoint_optimization(Nag=6, r_init=6, led_height = 15, target_r = 0, target_theta=0, I_tg=100, I0_initial=50)
        self.Pos_com = Pose()
        self.Pos_com.position.x = 0
        self.Pos_com.position.y = 0
        self.x_tg = 0
        self.y_tg = 0
        self.I_pwm_value = [0] * 6
        self.r_pwm_value = [0] * 6
        self.theta_pwm_value = [0] * 6
        
        rospy.Subscriber("/illumination_trajectory_generation", Pose, self.update_target_pos, queue_size=1)
        
    def update_target_pos(self, msg):
        
        self.x_tg = msg.position.x
        self.y_tg = msg.position.y
        
        self.optimized_illumination_results.x_tg = self.x_tg
        self.optimized_illumination_results.y_tg = self.y_tg
        
        opt_I0, opt_radii, opt_rotation = self.optimized_illumination_results.main()
        for i in range(6):
            self.I_pwm_value[i] = int((opt_I0[i] * 4095) / 4000)
            self.r_pwm_value[i] = int((opt_I0[i] * 4095) / 4000)
        print("optimum_illumination", opt_I0)
        print("optimum_radius", opt_I0)
        print("optimum_rotation", opt_rotation)
        
    def start(self):
        while not rospy.is_shutdown():
            continue
        
        
if __name__ == '__main__':
    #communication = RR_XY_TABLE(sys.argv[1],sys.argv[2])
    q_tj = Experiment_sim()
    q_tj.start()
        

        
        
        
        


