#!/usr/bin/env python3
# Author: Mohsen

import rospy
from std_msgs.msg import Float32MultiArray
import tkinter as tk

class ManualCommandGenerator:
    def __init__(self):
        rospy.init_node("manual_command_generator", anonymous=True)
        self.pub = rospy.Publisher("/commands", Float32MultiArray, queue_size=1)

        self.actuators = [2.0] * 6
        self.leds = [100] * 6

        self.root = tk.Tk()
        self.root.title("Actuators and LEDs")

        self.actuator_entries = []
        self.led_entries = []

        for i in range(6):
            tk.Label(self.root, text=f"Actuator {i+1}").grid(row=i, column=0)
            act_entry = tk.Entry(self.root, width=5)
            act_entry.insert(0, str(self.actuators[i]))
            act_entry.grid(row=i, column=1)
            self.actuator_entries.append(act_entry)

            tk.Label(self.root, text=f"LED {i+1}").grid(row=i, column=2)
            led_entry = tk.Entry(self.root, width=5)
            led_entry.insert(0, str(self.leds[i]))
            led_entry.grid(row=i, column=3)
            self.led_entries.append(led_entry)

        tk.Button(self.root, text="Update Values", command=self.update_values).grid(row=6, column=0, columnspan=4)

        self.root.after(100, self.publish_loop)

    def update_values(self):
        try:
            self.actuators = [float(entry.get()) for entry in self.actuator_entries]
            self.leds = [float(entry.get()) for entry in self.led_entries]
        except ValueError:
            print("Invalid input, must be numbers.")

    def publish_loop(self):
        if not rospy.is_shutdown():
            data = self.actuators + self.leds
            msg = Float32MultiArray(data=data)
            self.pub.publish(msg)
            self.root.after(1000, self.publish_loop)  # 1 Hz

    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    node = ManualCommandGenerator()
    node.run()
