#!/usr/bin/env python

import rospy
from uav import uav
from mavros_msgs.srv import SetMode

# Boilerplate for basic UAV hover

# Update rate
rate = 2 #60 times every second

class offboard_node():

    def __init__(self):
        print("Initalising Controller")

        self.uav = uav(setpoint_topic="/nightray/mavros/setpoint_position/local")

        rospy.wait_for_service("/nightray/mavros/set_mode") # Wait for service to initalise
        self.flight_mode_srv = rospy.ServiceProxy('/nightray/mavros/set_mode', SetMode)

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)
    
        while not rospy.is_shutdown():

            self.uav.setpoint(0,0,1) # Publish setpoint at x=0, y=0, z=1

            # if(self.flight_mode_srv.call(['base_mode', 'custom_mode'='OFFBOARD']).mode_sent == True):
            if(self.flight_mode_srv(custom_mode='OFFBOARD').mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            self.rosrate.sleep()

        
    def quit(self):
        print("Killing node")
        rospy.signal_shutdown("Node shutting down")


if __name__ == '__main__':
    
    rospy.init_node('Offboard_Node')

    node = offboard_node()

    rospy.spin()