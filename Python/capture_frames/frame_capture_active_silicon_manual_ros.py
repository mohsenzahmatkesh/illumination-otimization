#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 15 18:12:46 2025
@author: mohsen
"""

import cv2
import numpy as np
import os
import time
import subprocess
import rospy
from std_msgs.msg import Float32MultiArray

zoom = 7000
focus = 4798
exposure = 200
gain = 4

latest_command = None

def set_camera_controls():
    subprocess.run([
        "v4l2-ctl", "-d", "/dev/video0",
        "--set-ctrl=auto_exposure=1",
        f"--set-ctrl=zoom_absolute={zoom}",
        f"--set-ctrl=focus_automatic_continuous=0",
        f"--set-ctrl=focus_absolute={focus}",
        f"--set-ctrl=exposure_time_absolute={exposure}",
        f"--set-ctrl=gain={gain}"
    ])

def command_callback(msg):
    global latest_command
    latest_command = msg.data

set_camera_controls()

IMAGE_WIDTH = 3840
IMAGE_HEIGHT = 2160

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)

os.makedirs("captured_frames2", exist_ok=True)
existing_files = [f for f in os.listdir("captured_frames2") if f.endswith(".png")]
counter = len(existing_files) + 1

def enhance_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    s = cv2.add(s, 40)
    hsv_enhanced = cv2.merge((h, s, v))
    return cv2.cvtColor(hsv_enhanced, cv2.COLOR_HSV2BGR)

cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Camera", 1280, 720)

rospy.init_node("camera_capture_node", anonymous=True)
rospy.Subscriber("/commands", Float32MultiArray, command_callback)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        break

#    frame = enhance_colors(frame)
    frame = cv2.convertScaleAbs(frame, alpha=1.2, beta=15)
    cv2.imshow("Camera", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        if latest_command and len(latest_command) >= 12:
            actuator_vals = "_".join([f"{v:.2f}" for v in latest_command[:6]])
            led_vals = "_".join([f"{v:.0f}" for v in latest_command[6:]])
            filename = f"captured_frames2/actuator_{actuator_vals}_LED_{led_vals}.png"
        else:
            filename = f"captured_frames2/frame_{counter}.png"
        cv2.imwrite(filename, frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        print(f"Saved {filename}")
        counter += 1

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
