#!/usr/bin/env python

import rospy
from uav import uav, uav_variables

# Example code for a simple survey pattern

rate = 60 # Update rate

class offboard_node():

    def __init__(self):
        print("Initalising Simple Survey Node")

        self.uav = uav(survey_array_z=[[0.9,-1.9,1],[4.79,-1.9,1]]) # Initalise UAV object

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)
    
        while not rospy.is_shutdown():

            self.uav.continous_survey(threshold=0.25)
            self.rosrate.sleep()

    
    def quit(self):
        print("Killing node")
        rospy.signal_shutdown("Node shutting down")


if __name__ == '__main__':
    
    rospy.init_node('Offboard_Wall_Node')

    node = offboard_node()

    rospy.spin()
