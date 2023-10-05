#!/usr/bin/env python

import rospy
from uav import uav

# Boilerplate for basic UAV hover

# Update rate
rate = 60 #60 times every second

class offboard_node():

    def __init__(self):
        print("Initalising Controller")

        self.uav = uav()

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)
    
        while not rospy.is_shutdown():
            self.uav.setpoint(0,0,1) # Publish setpoint at x=0, y=0, z=1
            self.rosrate.sleep()

        
    def quit(self):
        print("Killing node")
        rospy.signal_shutdown("Node shutting down")


if __name__ == '__main__':
    
    rospy.init_node('Offboard_Node')

    node = offboard_node()

    rospy.spin()
