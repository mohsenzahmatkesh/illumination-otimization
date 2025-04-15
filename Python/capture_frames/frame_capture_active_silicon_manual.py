#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 15 18:12:46 2025

@author: mohsen
"""

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

# Fixed camera settings
zoom = 7000
focus = 4798
exposure = 200
gain = 4

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

# Apply fixed settings
set_camera_controls()

# Set resolution
IMAGE_WIDTH = 3840
IMAGE_HEIGHT = 2160

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)

# Create folder if needed and initialize filename counter
os.makedirs("captured_frames2", exist_ok=True)
existing_files = [f for f in os.listdir("captured_frames2") if f.startswith("frame_") and f.endswith(".png")]
counter = len(existing_files) + 1

# Optional color enhancement
def enhance_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    s = cv2.add(s, 40)
    hsv_enhanced = cv2.merge((h, s, v))
    return cv2.cvtColor(hsv_enhanced, cv2.COLOR_HSV2BGR)

# Create display window
cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Camera", 1280, 720)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Enhance frame for better visuals
    frame = enhance_colors(frame)
    frame = cv2.convertScaleAbs(frame, alpha=1.2, beta=15)

    cv2.imshow("Camera", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        filename = f"captured_frames2/frame_{counter}.png"
        cv2.imwrite(filename, frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        print(f"Saved {filename}")
        counter += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
