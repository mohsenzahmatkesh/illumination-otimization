#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 10 20:10:57 2025
@author: mohsen
"""

import cv2
import numpy as np
import os
import time
import subprocess

# Initial default values
zoom = 3000
focus = 4676
exposure = 200
gain = 4

# Limits from v4l2-ctl --all
ZOOM_MIN, ZOOM_MAX = 0, 32256
FOCUS_MIN, FOCUS_MAX = 3940, 5320
EXPOSURE_MIN, EXPOSURE_MAX = 1, 10000

def set_camera_controls(z, f, e):
    subprocess.run([
        "v4l2-ctl", "-d", "/dev/video0",
        "--set-ctrl=auto_exposure=1",
        f"--set-ctrl=zoom_absolute={z}",
        f"--set-ctrl=focus_automatic_continuous=0",
        f"--set-ctrl=focus_absolute={f}",
        f"--set-ctrl=exposure_time_absolute={e}",
        f"--set-ctrl=gain={gain}"
    ])

# Set initial values
set_camera_controls(zoom, focus, exposure)

IMAGE_WIDTH = 3840
IMAGE_HEIGHT = 2160

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
os.makedirs("captured_frames2", exist_ok=True)
existing_files = [f for f in os.listdir("captured_frames2") if f.startswith("frame_") and f.endswith(".png")]
counter = len(existing_files) + 1  # Start from next available number

def enhance_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    s = cv2.add(s, 40)
    hsv_enhanced = cv2.merge((h, s, v))
    return cv2.cvtColor(hsv_enhanced, cv2.COLOR_HSV2BGR)

# GUI Trackbars
cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Camera", 1280, 720)

def nothing(x): pass

cv2.createTrackbar("Zoom", "Camera", zoom, ZOOM_MAX, nothing)
cv2.createTrackbar("Focus", "Camera", focus, FOCUS_MAX, nothing)
cv2.createTrackbar("Exposure", "Camera", exposure, EXPOSURE_MAX, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = enhance_colors(frame)
    frame = cv2.convertScaleAbs(frame, alpha=1.2, beta=15)

    cv2.imshow("Camera", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        filename = f"captured_frames/frame_{counter}.png"
        cv2.imwrite(filename, frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])

        print(f"Saved {filename}")
        counter += 1
    elif key == ord('q'):
        break


    # Read slider values
    new_zoom = cv2.getTrackbarPos("Zoom", "Camera")
    new_focus = cv2.getTrackbarPos("Focus", "Camera")
    new_exposure = cv2.getTrackbarPos("Exposure", "Camera")

    # If values changed, update camera
    if new_zoom != zoom or new_focus != focus or new_exposure != exposure:
        zoom = new_zoom
        focus = new_focus
        exposure = new_exposure
        set_camera_controls(zoom, focus, exposure)

cap.release()
cv2.destroyAllWindows()

