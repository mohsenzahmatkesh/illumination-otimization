#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 15 12:08:35 2025
@author: Mohsen Zahmatkesh
"""

import numpy as np
import time
import rospy
from std_msgs.msg import Float32MultiArray
from ServoPi import PWM

pwm = PWM(address=0x42)  
pwm.set_pwm_freq(50)  
np.float = float 
np.int = int   

def map_range(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class rpi_experiment():
   
    def __init__(self):
        rospy.init_node("actuation")
        rospy.Subscriber("/commands", Float32MultiArray, self.led_actuators_update, queue_size=1)

        self.led_pwm = [0] * 6
        self.rdius_pwm = [0] * 6
        self.prev_led_pwm = [0] * 6  
        self.prev_rdius_pwm = [0] * 6  
        self.steps = 10
        
    def led_actuators_update(self, msg):
          data = msg.data
          self.rdius_pwm = data[:6]
          self.led_pwm = data[6:12]
      
          target_act_pwm = [map_range(val, 0, 2, 205, 410) for val in self.rdius_pwm]
          target_led_pwm = [map_range(val, 0, 100, 0, 4095) for val in self.led_pwm]
      
          step_act = [(target - current) / self.steps for target, current in zip(target_act_pwm, self.prev_rdius_pwm)]
          step_led = [(target - current) / self.steps for target, current in zip(target_led_pwm, self.prev_led_pwm)]
      
          for i in range(6):
              pwm.set_pwm(channel=i + 1, on_time=0, off_time=int(target_led_pwm[i]))
      
          for step in range(self.steps):
              for i in range(6):
                  pwm.set_pwm(channel=i + 7, on_time=0,
                              off_time=int(self.prev_rdius_pwm[i] + step * step_act[i]))
              time.sleep(0.01)
      
          for i in range(6):
              pwm.set_pwm(channel=i + 7, on_time=0, off_time=int(target_act_pwm[i]))
      
          self.prev_rdius_pwm = target_act_pwm
          self.prev_led_pwm = target_led_pwm
      
          print("Smoothed actuator PWM:", self.prev_rdius_pwm)
          print("Smoothed LED PWM:", self.prev_led_pwm)

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
    q_tj = rpi_experiment()
    q_tj.start()


        
        
        
        


