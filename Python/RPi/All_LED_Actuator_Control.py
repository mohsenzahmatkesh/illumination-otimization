#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import math
import time
from ServoPi import PWM

pwm = PWM(address=0x42)  
pwm.set_pwm_freq(50)  
previous_positions = {"d1": 205, "d2": 205, "d3": 205, "d4": 205, "d5": 205, "d6": 205}
position_threshold = 2  
steps = 10
delay=0.05

try 
    while True:
        
        pwm.set_pwm(channel=1, on_time=0, off_time=4095)
        pwm.set_pwm(channel=2, on_time=0, off_time=4095)
        pwm.set_pwm(channel=3, on_time=0, off_time=4095)
        pwm.set_pwm(channel=4, on_time=0, off_time=4095)
        pwm.set_pwm(channel=5, on_time=0, off_time=4095)
        pwm.set_pwm(channel=6, on_time=0, off_time=4095)
        for j in range(3):
            d1 = max(205, min(int(205 + (0.25 - 0.15) * 2050), 410))
            d2 = max(205, min(int(205 + (0.25 - 0.15) * 2050), 410))
            d3 = max(205, min(int(205 + (0.25 - 0.15) * 2050), 410))
            d4 = max(205, min(int(205 + (0.25 - 0.15) * 2050), 410))
            d5 = max(205, min(int(205 + (0.25 - 0.15) * 2050), 410))
            d6 = max(205, min(int(205 + (0.25 - 0.15) * 2050), 410))
        
        step_sizes = [
            (d1 - 205) / steps,
            (d2 - 205) / steps,
            (d3 - 205) / steps,
            (d4 - 205) / steps,
            (d5 - 205) / steps,
            (d6 - 205) / steps,
        ]

        for i in range(steps):
            pwm.set_pwm(channel=7, on_time=0, off_time=int(205 + step_sizes[0] * i))
            pwm.set_pwm(channel=8, on_time=0, off_time=int(205 + step_sizes[1] * i))
            pwm.set_pwm(channel=9, on_time=0, off_time=int(205 + step_sizes[2] * i))
            pwm.set_pwm(channel=10, on_time=0, off_time=int(205 + step_sizes[3] * i))
            pwm.set_pwm(channel=11, on_time=0, off_time=int(205 + step_sizes[4] * i))
            pwm.set_pwm(channel=12, on_time=0, off_time=int(205 + step_sizes[5] * i))
            time.sleep(delay)

        pwm.set_pwm(channel=7, on_time=0, off_time=int(d1))
        pwm.set_pwm(channel=8, on_time=0, off_time=int(d2))
        pwm.set_pwm(channel=9, on_time=0, off_time=int(d3))
        pwm.set_pwm(channel=10, on_time=0, off_time=int(d4))
        pwm.set_pwm(channel=11, on_time=0, off_time=int(d5))
        pwm.set_pwm(channel=12, on_time=0, off_time=int(d6))
        

except KeyboardInterrupt:
    print("\nWe are going home")
for i in range(1, 7):
    pwm.set_pwm(channel=i, on_time=0, off_time=0)  
for i in range(7, 13):
    pwm.set_pwm(channel=i, on_time=0, off_time=205)
