#!/usr/bin/env python3
# Author Mohsen
import rospy
from std_msgs.msg import Float32MultiArray

class ManualCommandGenerator:
    def __init__(self):
        rospy.init_node("manual_command_generator")
        self.pub = rospy.Publisher("/commands", Float32MultiArray, queue_size=1)

      
        self.actuators = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        self.leds = [100, 100, 100, 100, 100, 100]

    def publish_command(self):
    
        data = self.actuators + self.leds
        msg = Float32MultiArray(data=data)
        self.pub.publish(msg)

if __name__ == '__main__':
    node = ManualCommandGenerator()
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        node.publish_command()
        rate.sleep()
