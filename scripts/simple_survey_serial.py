#!/usr/bin/env python

import rospy
from uav import uav, uav_variables
import serial
import time
ser = serial.Serial('/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:CF:79:02:98:F0-if00', 115200) #ls /dev/serial/by-id/*

# Example code for a simple survey pattern

rate = 60 # Update rate

class offboard_node():

    def __init__(self):
        print("Initalising Simple Survey Node")

        self.uav = uav(survey_array_z=[[0.9,-1.9,1.7],[4.79,-1.9,1.7]]) # Initalise UAV object
        self.prev_msg=""

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)
    
        while not rospy.is_shutdown():

            self.uav.continous_survey(threshold=0.25)
            if self.uav.pos.x >= 3.08 and self.uav.pos.x <=3.68:
                print(self.uav.pos.x,self.uav.pos.y,self.uav.pos.z)
                rospy.logwarn_throttle_identical(1,"Releasing[%f,%f,%f]",self.uav.pos.x,self.uav.pos.y,self.uav.pos.z)
                self.write_serial("payload_drop")
            else:
                rospy.logwarn_throttle_identical(10,"Resetting")
                self.write_serial("payload_reset")
            self.rosrate.sleep()

    
    def write_serial(self,msg):
        if msg != self.prev_msg:
            ser.write(str.encode(str(msg)+ "\n"))
            self.prev_msg = msg
            rospy.logwarn("Sending Release")
            time.sleep(0.075) # Needed for ESP32C3 to read consecutive serial commands

    def quit(self):
        print("Killing node")
        rospy.signal_shutdown("Node shutting down")


if __name__ == '__main__':
    
    rospy.init_node('Offboard_Wall_Node')

    node = offboard_node()

    rospy.spin()
