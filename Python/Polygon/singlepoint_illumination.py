#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 17 10:39:27 2025

@author: frekabi
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 14 13:26:44 2025

@author: mohsen
"""

import numpy as np
np.float = float  # Add this line
np.int = int      # (Optional) if you hit similar errors with np.int
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from mpl_toolkits.mplot3d import Axes3D


class SinglePoint_Illumination():
    
    def __init__(self, Nag=6, r_init=6, led_height = 15, target_r = 0, target_theta=0, I_tg=100, I0_initial=50):
        
        
        self.Nag = Nag
        self.gamma = 1.0
        self.theta_c = np.deg2rad(30)
        self.I0_initial = np.ones(self.Nag) * I0_initial
        self.r_initial = np.ones(self.Nag) * r_init  # cm
        self.base_angles = base_angles = np.linspace(0, (Nag - 1) * 2 * np.pi / Nag, Nag) # np.deg2rad(np.array([0, 60, 120, 180, 240, 300]))
        self.led_height = led_height # cm
        self.target_r = target_r  # cm
        self.target_theta = np.deg2rad(target_theta)
        self.I_tg = I_tg
        self.x_tg = target_r * np.cos(target_theta)
        self.y_tg = target_r * np.sin(target_theta)
        self.initial_guess = np.concatenate([
            self.I0_initial * (1 + (2 * np.random.rand(Nag) - 1)),
            self.r_initial,
            [0]
        ])
        
        self.lb = np.zeros(self.Nag * 2 + 1)
        self.ub = np.full(self.Nag * 2 + 1, np.inf)
        self.lb[:self.Nag] = 0
        self.ub[:self.Nag] = 4000
        self.lb[self.Nag:self.Nag*2] = 1
        self.ub[self.Nag:self.Nag*2] = 8
        self.lb[self.Nag*2] = 0
        self.ub[self.Nag*2] = 2 * np.pi

        # Create bounds list for scipy
        self.bounds = [(l, u) for l, u in zip(self.lb, self.ub)]
        
    def objective_func(self,x):
        I0s = x[:self.Nag]
        radii = x[self.Nag:self.Nag*2]
        delta_theta = x[self.Nag*2]
        
        total_I = self.compute_intensity(I0s, radii, delta_theta)
        
        # Regularization terms
        reg_radii = 0.001 * np.sum((radii - self.r_initial)**2)
        reg_I0 = 0.0 * np.sum((I0s - self.I0_initial)**2)
        
        return (self.I_tg - total_I)**2 + reg_radii + reg_I0
    
    
    def compute_intensity(self,I0s, radii, delta_theta):
        total_I = 0.0
        for i in range(self.Nag):
            # LED position
            theta = self.base_angles[i] + delta_theta
            x_led = radii[i] * np.cos(theta)
            y_led = radii[i] * np.sin(theta)
            
            # Vector components
            dx = self.x_tg - x_led
            dy = self.y_tg - y_led
            dz = -self.led_height
            
            # Distance and angle calculations
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            cos_theta = -dz / distance
            theta_angle = np.arccos(np.clip(cos_theta, -1, 1))
            
            # Intensity contribution
            total_I += self.gamma * I0s[i] * np.exp(-(theta_angle/self.theta_c)**2) / distance**2
        return total_I
    
    
    def plot_results(self,opt_angles,opt_radii,opt_I0):
        # Polar plot
        plt.figure()
        for i in range(self.Nag):
            plt.polar([self.base_angles[i], opt_angles[i]], [self.r_initial[i], opt_radii[i]], 'k--')
        plt.scatter(self.base_angles, self.r_initial, c='b', label='Initial')
        plt.scatter(opt_angles, opt_radii, c='r', marker='x', label='Optimized')
        plt.scatter(self.target_theta, self.target_r, c='g', marker='*', s=200, label='Target')
        plt.legend()
        plt.title('Optimized LED Configuration')
        
        # Intensity comparison
        fig, (ax1, ax2) = plt.subplots(1, 2)
        ax1.bar(range(self.Nag), self.I0_initial, color='gray')
        ax1.set_title('Initial Intensities')
        ax1.set_ylabel('I_0')
        
        ax2.bar(range(self.Nag), opt_I0, color='blue')
        ax2.set_title('Optimized Intensities')
        ax2.set_ylabel('I_0')
        
        # 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Initial positions
        xi = self.r_initial * np.cos(self.base_angles)
        yi = self.r_initial * np.sin(self.base_angles)
        ax.scatter(xi, yi, np.ones(self.Nag)*self.led_height, c='b', label='Initial')
        
        # Optimized positions
        xo = opt_radii * np.cos(opt_angles)
        yo = opt_radii * np.sin(opt_angles)
        ax.scatter(xo, yo, np.ones(self.Nag)*self.led_height, c='r', marker='x', label='Optimized')
        
        # Target
        ax.scatter([self.x_tg], [self.y_tg], [0], c='g', marker='*', s=200, label='Target')
        
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_zlabel('Z (cm)')
        ax.legend()
        plt.title('3D System Configuration')
        
        plt.show()
    
    
    def main(self):
        
        result = minimize(self.objective_func, self.initial_guess, method='SLSQP', bounds=self.bounds,
                          options={'maxiter': 10000, 'disp': True})
        
        x_opt = result.x
        opt_I0 = x_opt[:self.Nag]
        opt_radii = x_opt[self.Nag:self.Nag*2]
        opt_rotation = x_opt[self.Nag*2]
        opt_angles = self.base_angles + opt_rotation

        # Calculate final intensity
        final_I = self.compute_intensity(opt_I0, opt_radii, opt_rotation)
        print(f"Achieved Intensity: {final_I}")
        print(f"Target Intensity: {self.I_tg}")
        print(f"Error: {abs(self.I_tg - final_I)}")
        
        # self.plot_results(opt_angles,opt_radii,opt_I0)
        
        return opt_I0, opt_radii, opt_rotation
        

if __name__ == "__main__":
   sing_Illum = SinglePoint_Illumination(Nag=6, r_init=6, led_height = 15, target_r = 0, target_theta=0, I_tg=100, I0_initial=50)
   sing_Illum.main()


