#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 17 12:08:35 2025

@author: Mohsen Zahmatkesh
"""
import numpy as np
import time
from singlepoint_illumination_main import singlepoint_optimization
import rospy
from geometry_msgs.msg import Pose
from ServoPi import PWM
pwm = PWM(address=0x42)  
pwm.set_pwm_freq(50)  
np.float = float 
np.int = int     

class Experiment_sim():
    
    def __init__(self):
        
        rospy.init_node("illumination_command_generation")
        
        self.optimized_illumination_results = singlepoint_optimization(
                Nag=6, r_init=6, led_height = 15, target_r = 0, target_theta=0, I_tg=100, I0_initial=50)
        
        self.Pos_com = Pose()
        self.Pos_com.position.x = 0
        self.Pos_com.position.y = 0
        self.x_tg = 0
        self.y_tg = 0
        
        self.I_pwm_value = [0] * 6
        self.r_pwm_value = [0] * 6
        self.theta_pwm_value = [0] * 6
        self.d = [205] * 6
        
        self.prev_I_pwm_value = [0] * 6  
        self.prev_r_pwm_value = [0] * 6  
        self.prev_d = [205] * 6
        
        
        self.step_sizes = [0] * 6
        self.steps = 10
        
        rospy.Subscriber("/illumination_trajectory_generation", Pose, self.update_target_pos, queue_size=1)
        
    def update_target_pos(self, msg):
        
        self.x_tg = msg.position.x
        self.y_tg = msg.position.y
        
        self.optimized_illumination_results.x_tg = self.x_tg
        self.optimized_illumination_results.y_tg = self.y_tg
        
        opt_I0, opt_radii, opt_rotation = self.optimized_illumination_results.main()
        
        print("optimum_illumination_sim", opt_I0)
        print("optimum_radius_sim", opt_radii)
        print("optimum_rotation_sim", opt_rotation)
        
        for i in range(6):
            
            self.I_pwm_value[i] = int((opt_I0[i] * 4095) / 4000)
            self.r_pwm_value[i] = 0.15 - ((opt_radii[i] - 1) / (8 - 1)) * -(0.25 - 0.15)
            self.d[i] = max(205, min(int(205 + (self.r_pwm_value[i] - 0.15) * 2050), 410))
            self.step_sizes[i] = int((self.d[i] - 205) / self.steps)

            
        #LEDs
        pwm.set_pwm(channel=1, on_time=0, off_time=self.I_pwm_value[0])
        pwm.set_pwm(channel=2, on_time=0, off_time=self.I_pwm_value[1])
        pwm.set_pwm(channel=3, on_time=0, off_time=self.I_pwm_value[2])
        pwm.set_pwm(channel=4, on_time=0, off_time=self.I_pwm_value[3])
        pwm.set_pwm(channel=5, on_time=0, off_time=self.I_pwm_value[4])
        pwm.set_pwm(channel=6, on_time=0, off_time=self.I_pwm_value[5])
        
        #Actuators
        for i in range(self.steps):
            pwm.set_pwm(channel=7, on_time=0, off_time=(self.prev_d[0] + self.step_sizes[0] * i))
            pwm.set_pwm(channel=8, on_time=0, off_time=(self.prev_d[1] + self.step_sizes[1] * i))
            pwm.set_pwm(channel=9, on_time=0, off_time=(self.prev_d[2] + self.step_sizes[2] * i))
            pwm.set_pwm(channel=10, on_time=0, off_time=(self.prev_d[3] + self.step_sizes[3] * i))
            pwm.set_pwm(channel=11, on_time=0, off_time=(self.prev_d[4] + self.step_sizes[4] * i))
            pwm.set_pwm(channel=12, on_time=0, off_time=(self.prev_d[5] + self.step_sizes[5] * i))
            
            time.sleep(0.05)
            
        pwm.set_pwm(channel=7, on_time=0, off_time=int(self.d[0]))
        pwm.set_pwm(channel=8, on_time=0, off_time=int(self.d[1]))
        pwm.set_pwm(channel=9, on_time=0, off_time=int(self.d[2]))
        pwm.set_pwm(channel=10, on_time=0, off_time=int(self.d[3]))
        pwm.set_pwm(channel=11, on_time=0, off_time=int(self.d[4]))
        pwm.set_pwm(channel=12, on_time=0, off_time=int(self.d[5]))
        
        self.prev_I_pwm_value = self.I_pwm_value[:]
        self.prev_r_pwm_value = self.r_pwm_value[:]
        self.prev_d = self.d[:]


        print("optimum_illumination_rpi", self.I_pwm_value)
        print("optimum_radius_rpi", self.d)
        print("optimum_rotation_rpi", opt_rotation)
        


    def start(self):
        while not rospy.is_shutdown():
            continue
            
        if KeyboardInterrupt:
            print("\nShutting down: Turning off all LEDs and actuators.")
            for i in range(1, 7): 
                pwm.set_pwm(channel=i, on_time=0, off_time=0)
            for i in range(7, 13):  
                pwm.set_pwm(channel=i, on_time=0, off_time=410)
            print("All LEDs and actuators are now OFF.")


        
        
if __name__ == '__main__':
    #communication = RR_XY_TABLE(sys.argv[1],sys.argv[2])
    q_tj = Experiment_sim()
    q_tj.start()
        

        
        
        
        


