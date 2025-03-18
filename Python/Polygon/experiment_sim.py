#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 17 12:08:35 2025

@author: frekabi
"""
import numpy as np
np.float = float  # Add this line
np.int = int      # (Optional) if you hit similar errors with np.int
from singlepoint_illumination import SinglePoint_Illumination
import rospy
from geometry_msgs.msg import Pose

class Experiment_sim():
    
    def __init__(self):
        
        rospy.init_node("illum_ctrl")
        
        self.illum_optim = SinglePoint_Illumination(Nag=6, r_init=6, led_height = 15, target_r = 0, target_theta=0, I_tg=100, I0_initial=50)
        
        self.Pos_com = Pose()
        self.Pos_com.position.x = 0
        self.Pos_com.position.y = 0
        
        self.x_tg = 0
        self.y_tg = 0
        
        rospy.Subscriber("/illum_traj", Pose, self.led_illum_tgt, queue_size=1)
        
    def led_illum_tgt(self,msg):
        
        self.x_tg = msg.position.x
        self.y_tg = msg.position.y
        
        self.illum_optim.x_tg = self.x_tg
        self.illum_optim.y_tg = self.y_tg
        
        opt_I0, opt_radii, opt_rotation = self.illum_optim.main()
        
        
    def start(self):
        while not rospy.is_shutdown():
            continue
        
        
if __name__ == '__main__':
    #communication = RR_XY_TABLE(sys.argv[1],sys.argv[2])
    q_tj = Experiment_sim()
    q_tj.start()
        

        
        
        
        


