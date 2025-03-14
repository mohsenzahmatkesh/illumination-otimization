#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 14 13:26:44 2025

@author: mohsen
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from mpl_toolkits.mplot3d import Axes3D

# LED Parameters
gamma = 1.0
theta_c = np.deg2rad(30)
I0_initial = np.ones(6)
r_initial = np.ones(6) * 6  # cm
base_angles = np.deg2rad(np.array([0, 60, 120, 180, 240, 300]))
led_height = 15  # cm

# Target Parameters
target_r = 0.0  # cm
target_theta = np.deg2rad(0)
I_tg = 100

# Convert target to Cartesian
x_tg = target_r * np.cos(target_theta)
y_tg = target_r * np.sin(target_theta)

# Optimization variables: [I0s(6); radii(6); delta_theta(1)]
initial_guess = np.concatenate([
    I0_initial * (1 + (2 * np.random.rand(6) - 1)),
    r_initial,
    [0]
])

# Bounds
lb = np.zeros(13)
ub = np.full(13, np.inf)
lb[:6] = 0
ub[:6] = 4000
lb[6:12] = 1
ub[6:12] = 8
lb[12] = 0
ub[12] = 2 * np.pi

# Create bounds list for scipy
bounds = [(l, u) for l, u in zip(lb, ub)]

def objective_func(x):
    I0s = x[:6]
    radii = x[6:12]
    delta_theta = x[12]
    
    total_I = compute_intensity(I0s, radii, delta_theta)
    
    # Regularization terms
    reg_radii = 0.001 * np.sum((radii - r_initial)**2)
    reg_I0 = 0.001 * np.sum((I0s - I0_initial)**2)
    
    return (I_tg - total_I)**2 + reg_radii + reg_I0

def compute_intensity(I0s, radii, delta_theta):
    total_I = 0.0
    for i in range(6):
        # LED position
        theta = base_angles[i] + delta_theta
        x_led = radii[i] * np.cos(theta)
        y_led = radii[i] * np.sin(theta)
        
        # Vector components
        dx = x_tg - x_led
        dy = y_tg - y_led
        dz = -led_height
        
        # Distance and angle calculations
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        cos_theta = -dz / distance
        theta_angle = np.arccos(np.clip(cos_theta, -1, 1))
        
        # Intensity contribution
        total_I += gamma * I0s[i] * np.exp(-(theta_angle/theta_c)**2) / distance**2
    return total_I

# Run optimization
result = minimize(objective_func, initial_guess, method='SLSQP', bounds=bounds,
                  options={'maxiter': 10000, 'disp': True})

# Extract results
x_opt = result.x
opt_I0 = x_opt[:6]
opt_radii = x_opt[6:12]
opt_rotation = x_opt[12]
opt_angles = base_angles + opt_rotation

# Calculate final intensity
final_I = compute_intensity(opt_I0, opt_radii, opt_rotation)
print(f"Achieved Intensity: {final_I}")
print(f"Target Intensity: {I_tg}")
print(f"Error: {abs(I_tg - final_I)}")

# Visualization
def plot_results():
    # Polar plot
    plt.figure()
    for i in range(6):
        plt.polar([base_angles[i], opt_angles[i]], [r_initial[i], opt_radii[i]], 'k--')
    plt.scatter(base_angles, r_initial, c='b', label='Initial')
    plt.scatter(opt_angles, opt_radii, c='r', marker='x', label='Optimized')
    plt.scatter(target_theta, target_r, c='g', marker='*', s=200, label='Target')
    plt.legend()
    plt.title('Optimized LED Configuration')
    
    # Intensity comparison
    fig, (ax1, ax2) = plt.subplots(1, 2)
    ax1.bar(range(6), I0_initial, color='gray')
    ax1.set_title('Initial Intensities')
    ax1.set_ylabel('I_0')
    
    ax2.bar(range(6), opt_I0, color='blue')
    ax2.set_title('Optimized Intensities')
    ax2.set_ylabel('I_0')
    
    # 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Initial positions
    xi = r_initial * np.cos(base_angles)
    yi = r_initial * np.sin(base_angles)
    ax.scatter(xi, yi, np.ones(6)*led_height, c='b', label='Initial')
    
    # Optimized positions
    xo = opt_radii * np.cos(opt_angles)
    yo = opt_radii * np.sin(opt_angles)
    ax.scatter(xo, yo, np.ones(6)*led_height, c='r', marker='x', label='Optimized')
    
    # Target
    ax.scatter([x_tg], [y_tg], [0], c='g', marker='*', s=200, label='Target')
    
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.legend()
    plt.title('3D System Configuration')
    
    plt.show()

plot_results()