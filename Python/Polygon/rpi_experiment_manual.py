"""
Created on Mon Mar 17 12:08:35 2025

@author: Mohsen Zahmatkesh
"""

import numpy as np
import time
from ServoPi import PWM

# Legacy compatibility
np.float = float
np.int = int

pwm = PWM(address=0x42)
pwm.set_pwm_freq(50)


class ManualIlluminationExperiment:
    def __init__(self, opt_I0, opt_radii, opt_rotation):
        self.I_pwm_value = [0] * 6
        self.r_pwm_value = [0] * 6
        self.d = [205] * 6
        self.angle_pwm_value = 0
        self.prev_I_pwm_value = [0] * 6
        self.prev_r_pwm_value = [0] * 6
        self.prev_d = [205] * 6
        self.prev_angle_pwm_value = 0
        self.step_sizes = [0] * 6
        self.steps = 10
        self.opt_I0 = opt_I0
        self.opt_radii = opt_radii
        self.opt_rotation = opt_rotation

    def run(self):
        print("Manual optimum illumination values:", self.opt_I0)
        print("Manual optimum radii:", self.opt_radii)
        print("Manual optimum rotation:", self.opt_rotation)

        for i in range(6):
            self.I_pwm_value[i] = int((self.opt_I0[i] * 4095) / 4000)
            self.r_pwm_value[i] = 0.15 - ((self.opt_radii[i] - 1) / (8 - 1)) * -(0.25 - 0.15)
            self.d[i] = max(205, min(int(205 + (self.r_pwm_value[i] - 0.15) * 2050), 410))
            self.angle_pwm_value = int((self.opt_rotation * 4095) / 360)
            self.step_sizes[i] = int((self.d[i] - 205) / self.steps)

        #LEDs
        pwm.set_pwm(channel=1, on_time=0, off_time=self.I_pwm_value[0])
        pwm.set_pwm(channel=2, on_time=0, off_time=self.I_pwm_value[1])
        pwm.set_pwm(channel=3, on_time=0, off_time=self.I_pwm_value[2])
        pwm.set_pwm(channel=4, on_time=0, off_time=self.I_pwm_value[3])
        pwm.set_pwm(channel=5, on_time=0, off_time=self.I_pwm_value[4])
        pwm.set_pwm(channel=6, on_time=0, off_time=self.I_pwm_value[5])

        #Linear Actuators
        for i in range(self.steps):
            pwm.set_pwm(channel=7, on_time=0, off_time=(self.prev_d[0] + self.step_sizes[0] * i))
            pwm.set_pwm(channel=8, on_time=0, off_time=(self.prev_d[1] + self.step_sizes[1] * i))
            pwm.set_pwm(channel=9, on_time=0, off_time=(self.prev_d[2] + self.step_sizes[2] * i))
            pwm.set_pwm(channel=10, on_time=0, off_time=(self.prev_d[3] + self.step_sizes[3] * i))
            pwm.set_pwm(channel=11, on_time=0, off_time=(self.prev_d[4] + self.step_sizes[4] * i))
            pwm.set_pwm(channel=12, on_time=0, off_time=(self.prev_d[5] + self.step_sizes[5] * i))
            
            time.sleep(0.05)
            
        pwm.set_pwm(channel=7, on_time=0, off_time=int(self.d[0]))
        pwm.set_pwm(channel=8, on_time=0, off_time=int(self.d[1]))
        pwm.set_pwm(channel=9, on_time=0, off_time=int(self.d[2]))
        pwm.set_pwm(channel=10, on_time=0, off_time=int(self.d[3]))
        pwm.set_pwm(channel=11, on_time=0, off_time=int(self.d[4]))
        pwm.set_pwm(channel=12, on_time=0, off_time=int(self.d[5]))
        
        #Rotation Actuator
        for i in range(self.steps):
            pwm.set_pwm(channel=13, on_time=0, off_time=(self.prev_angle_pwm_value + self.step_sizes[0] * i))
            
        pwm.set_pwm(channel=13, on_time=0, off_time=self.angle_pwm_value)    

        self.prev_I_pwm_value = self.I_pwm_value[:]
        self.prev_r_pwm_value = self.r_pwm_value[:]
        self.prev_d = self.d[:]
        self.prev_angle_pwm_value = self.angle_pwm_value

        print("Final PWM values for LEDs:", self.I_pwm_value)
        print("Final PWM values for actuators:", self.d)
        print("Rotation info:", self.angle_pwm_value)


if __name__ == '__main__':

    opt_I0 = [300, 280, 290, 310, 305, 295]          
    opt_radii = [2.5, 2.6, 2.4, 2.7, 2.5, 2.6]       
    opt_rotation = 30        

    experiment = ManualIlluminationExperiment(opt_I0, opt_radii, opt_rotation)

    try:
        start_time = time.time()
        while time.time() - start_time < 60:
            experiment.run()
    except KeyboardInterrupt:
        print("\nShutting down: Turning off all LEDs and actuators.")
        for i in range(1, 7):
            pwm.set_pwm(channel=i, on_time=0, off_time=0)
        for i in range(7, 13):
            pwm.set_pwm(channel=i, on_time=0, off_time=410)
        pwm.set_pwm(channel=13, on_time=0, off_time=0)
        print("All LEDs and actuators are now OFF.")



        

        
        
        
        


