#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 14 16:00:32 2025

@author: mohsen
"""

from ServoPi import PWM
import time

pwm = PWM(address=0x42)
pwm.set_pwm_freq(50)

channel = 13  # Your servo channel

def angle_to_pwm(angle):
    return int(205 + (angle / 180.0) * (410 - 205))

# Gradually move from 0° to 180°
for angle in range(0, 181, 1):  # Step = 1 degree
    pwm.set_pwm(channel, 0, angle_to_pwm(angle))
    time.sleep(0.02)  # 20ms delay for smooth motion

# Hold position briefly
time.sleep(1)

# Gradually move back from 180° to 0°
for angle in range(180, -1, -1):
    pwm.set_pwm(channel, 0, angle_to_pwm(angle))
    time.sleep(0.02)
