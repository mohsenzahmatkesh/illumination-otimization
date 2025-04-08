#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 17 11:28:00 2025

@author: frekabi
"""
import pandas as  pd
import time
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Twist 
import tty, termios
import sys, select, os
# filename = "/home/pannell/PhD/RoboRoyal/RoboRoyal_ws/src/RR_xytable/scripts/queen.txt"
# filename = "queen.txt"
# f = open("queen.txt", "r")


class Queen_Cam_TGT:
    
    def __init__(self, filename = 'h2xy_queen_2024-07-03-01-00-04_xy_0_queen.csv'):
        
        rospy.init_node("illumination_trajectory_generation")
        
        
        self.rate = rospy.Rate(10)
        self.data = pd.read_csv(filename)
        
        size = self.data.shape
        print(size)
        self.data_num = size[1]

        self.size = self.data.shape
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.MA_motion_pub = rospy.Publisher("/illumination_trajectory_generation", Pose, queue_size=2)
        
        self.Pos_com = Pose()
        
        self.Pos_com.position.x = 0
        self.Pos_com.position.y = 0
        
        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            self.key = sys.stdin.read(1)
        else:
            self.key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
    def main(self):
        
        tt = 0
        for i in range(self.size[0]):
                
            self.getKey()
            
            if self.key == '':
                
                queen_x = self.data.iat[i,10]
                queen_y = self.data.iat[i,11]
                
                cam_x = self.data.iat[i,3]
                cam_y = self.data.iat[i,4]
                
                self.Pos_com.position.x = queen_x - cam_x
                self.Pos_com.position.y = queen_y - cam_y
                
                self.MA_motion_pub.publish(self.Pos_com)
                
                
            else:
                break
                
            time.sleep(0.1)
            tt = tt + 0.01
        #print('time is %f sec'%(tt))
            
            
    def min_max(self,signal,val_min,val_max):
        
        if signal > val_max:
            signal = val_max
        elif signal < val_min:
            signal = val_min
            
        return signal
    
if __name__ == '__main__':
    #communication = RR_XY_TABLE(sys.argv[1],sys.argv[2])
    q_tj = Queen_Cam_TGT(filename = 'h2xy_queen_2024-07-03-01-00-04_xy_0_queen.csv')
    q_tj.main()
    