#!/usr/bin/env python

import rospy
from uav import uav, uav_variables
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage
import tf
from math import degrees
from transforms3d import _gohlketransforms,euler
import serial
from sensor_msgs.msg import Range
import time
import coordinates # Replace this with your own file
import tf2_ros

# 1) the egm96-5.pgm file from geographiclib.
# To get it on Ubuntu run:
# sudo apt install geographiclib-tools
# sudo geographiclib-get-geoids egm96-5
#
# 2) PyGeodesy
# To get it using pip:
# pip install PyGeodesy
from pygeodesy.geoids import GeoidPGM

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

rate = 60 # Update rate

# For alignment of camera_frame to drone_frame(CG), in m
cameratobody_x = 0 # +ve is forward
payload_drop_height=0.6

# Camera Topic for desired setpoint
camera_setpoint_topic="/tf"
camera_frame_id="/pole"
base_frame_id="/base_link"

# Threshold for jogging, when setpoint is under these conditions, drone will jog instead
threshold_jog=0.1 #m
threshold_jog_deg=10 #deg
max_deployment_times = 1

# ser = serial.Serial('/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:CF:79:02:99:0C-if00', 115200) #ls /dev/serial/by-id/*
# ser = serial.Serial('/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_F4:12:FA:D8:DA:58-if00', 115200) #ls /dev/serial/by-id/*

class offboard_node():

    def __init__(self):
        print("Initalising Offboard Waypoint Drop Node")

        self.uav = uav(survey_array=coordinates.waypoint_array) # Initalise UAV object
        self.uav.init_controller("close",1,0.125,1,0.125,1,0.8,0.5,0.0625) # Initalise additional controllers
        self.camera_setpoint = uav_variables() # Initalise a set of variables to store camera setpoints

        self.setpoint_latitude=coordinates.latitude
        self.setpoint_longitude=coordinates.longitude
        # self.setpoint_altitude=coordinates.altitude

        if camera_setpoint_topic != "/tf":
            rospy.Subscriber(
                camera_setpoint_topic,
                PoseStamped,
                self.camera_listener_callback)
        else:
            # rospy.Subscriber(
            #     camera_setpoint_topic,
            #     TFMessage,
            #     self.camera_listener_callback)
            print("Using TF Transforms for setpoints")
            self.listener = tf.TransformListener()
        
        print("Using " + camera_setpoint_topic + " setpoint topic")

        camera_setpoint_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/mavros/local_position/pose",PoseStamped,self.local_pos_callback)
        self.pos=uav_variables()
        self.prev_msg=""

        self.reset_timer=time.time()
        self.reset_dur=1
        self.stage="survey"

        deployment_times = 0
        self.detected = False

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)

        rospy.Subscriber("/mavros/global_position/global",NavSatFix,self.global_pos_callback)


        while not rospy.is_shutdown():


            # Constantly poll to see if transform is found to object and align to it thereafter
            try:
                (trans,rot)=self.listener.lookupTransform(camera_frame_id, base_frame_id, rospy.Time(0))

                rospy.loginfo_once("Detected Transform from camera")

                self.camera_setpoint.x = trans[0]+self.uav.pos.x
                self.camera_setpoint.y = trans[1]+self.uav.pos.y
                self.camera_setpoint.z = trans[2]+self.uav.pos.z + payload_drop_height
                self.camera_setpoint.rx = rot[0]*self.uav.pos.rx
                self.camera_setpoint.ry = rot[1]*self.uav.pos.ry
                self.camera_setpoint.rz = rot[2]*self.uav.pos.rz
                self.camera_setpoint.rw = rot[3]*self.uav.pos.rw
                self.detected = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logdebug("Missing tf transform")

            # Send out position for visualisation
            self.final_transform= tf2_ros.TransformStamped()
            self.final_transform.header.frame_id = "map"
            self.final_transform.header.stamp = rospy.get_rostime()
            self.final_transform.child_frame_id = "camera_setpoint"
            self.final_transform.transform.translation.x = self.camera_setpoint.x
            self.final_transform.transform.translation.y = self.camera_setpoint.y
            self.final_transform.transform.translation.z = self.camera_setpoint.z
            self.final_transform.transform.rotation.x  = self.camera_setpoint.rx
            self.final_transform.transform.rotation.y  = self.camera_setpoint.ry
            self.final_transform.transform.rotation.z  = self.camera_setpoint.rz
            self.final_transform.transform.rotation.w  = self.camera_setpoint.rw
            # camera_setpoint_broadcaster.sendTransform(self.final_transform) #TODO TEST

            current_yaw=euler.quat2euler([self.uav.pos.rw,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz])[2] #wxyz default
            setpoint_yaw=euler.quat2euler([self.camera_setpoint.rw,self.camera_setpoint.rx,self.camera_setpoint.ry,self.camera_setpoint.rz])[2] #wxyz default


            # # No setpoint sent yet
            # if self.camera_setpoint.x == 0 and self.camera_setpoint.y ==0 and self.camera_setpoint.z ==0:
            #     self.uav.setpoint_quat(self.uav.pos.x,self.uav.pos.y,self.uav.pos.z,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz,self.uav.pos.rw) #callback local position
            
            # Camera detected droppoint, switching from GPS to local setpoint mode
            if self.detected == True :
                rospy.logwarn_once("Detected Transform")
                self.uav.setpoint_controller(self.camera_setpoint,"close")
                rospy.loginfo_throttle_identical(2,"UAV moving to Camera Setpoint[%s,%s,%s] from [%s,%s,%s]",round(self.camera_setpoint.x,2),round(self.camera_setpoint.y,2),round(self.camera_setpoint.z,2),round(self.uav.pos.x,2),round(self.uav.pos.y,2),round(self.uav.pos.z,2))
                # At drop point, dropping payload
                if abs(self.camera_setpoint.x - self.uav.pos.x) < threshold_jog and abs(self.camera_setpoint.y-self.uav.pos.y) < threshold_jog and abs(self.camera_setpoint.z-self.uav.pos.z) < threshold_jog and degrees(abs(setpoint_yaw-current_yaw)) << threshold_jog_deg:
                    rospy.loginfo_throttle_identical(2,"UAV At Camera Setpoint[%s,%s,%s], dropping",self.camera_setpoint.x,self.camera_setpoint.y,self.camera_setpoint.z)
                    if deployment_times <max_deployment_times:
                        if (self.stage=="disarmed" or self.stage=="survey"):
                            rospy.loginfo_throttle_identical(2,"Releasing Payload")
                            self.stage="payload_drop"
                            self.write_serial(self.stage)
                            self.reset_timer=rospy.get_time()
                        if (self.stage=="payload_drop" and time.time()>=self.reset_timer+self.reset_dur):
                            rospy.loginfo_throttle_identical(2,"Disarming")
                            # self.stage="payload_reset"
                            # self.write_serial(self.stage)
                            self.stage="disarmed"
                            self.write_serial(self.stage)
                            deployment_times +=1
                    else:
                        rospy.loginfo_once("Deployment over")
 
            # Drone not at GPS Setpoint, send global coordinates
            else:
                if self.stage=="disarmed":
                    self.uav.setpoint_global(self.uav.global_pos.x, self.uav.global_pos.y, self.uav.global_pos.z-_egm96.height(self.uav.global_pos.x, self.uav.global_pos.y)) # GPS Altitude doesnt seem to be stable, so just hover at current height (with conversion)
                    rospy.loginfo_throttle_identical(1,"Sending GPS Setpoint[%s,%s,%s]", self.uav.global_pos.x, self.uav.global_pos.y, self.uav.global_pos.z-_egm96.height(self.uav.global_pos.x, self.uav.global_pos.y))
                elif self.stage=="survey":
                    self.uav.survey()
                    rospy.loginfo_throttle_identical(1,"On GPS Survey Setpoint at [%s,%s,%s]", self.uav.global_pos.x, self.uav.global_pos.y, self.uav.global_pos.z-_egm96.height(self.uav.global_pos.x, self.uav.global_pos.y))

            self.rosrate.sleep()


    def camera_listener_callback(self, msg):
        rospy.loginfo("New Camera setpoint(x:"+str(msg.pose.position.x)+", y:"+str(msg.pose.position.y)+", z:"+str(msg.pose.position.z)+")")
        # If TF is used as the as the position
        # if msg._type=="tf2_msgs/TFMessage":
        #     print(msg.transforms[0].header.frame_id,msg.transforms[0].child_frame_id)
        #     if msg.transforms[0].header.frame_id == base_frame_id and msg.transforms[0].child_frame_id == camera_frame_id:
        #         self.camera_setpoint.x = msg.transforms[0].transform.translation.x
        #         self.camera_setpoint.y = msg.transforms[0].transform.translation.y
        #         self.camera_setpoint.z = msg.transforms[0].transform.translation.z
        #         self.camera_setpoint.rx = msg.transforms[0].transform.rotation.x
        #         self.camera_setpoint.ry = msg.transforms[0].transform.rotation.y
        #         self.camera_setpoint.rz = msg.transforms[0].transform.rotation.z
        #         self.camera_setpoint.rw = msg.transforms[0].transform.rotation.w
        if msg._type=="geometry_msgs/PoseStamped":
            if self.camera_setpoint.z + payload_drop_height > 0:
                self.camera_setpoint.x = msg.pose.position.x
                self.camera_setpoint.y = msg.pose.position.y
                self.camera_setpoint.z = msg.pose.position.z + payload_drop_height
                self.camera_setpoint.rw = msg.pose.orientation.w
                self.camera_setpoint.rx = msg.pose.orientation.x
                self.camera_setpoint.ry = msg.pose.orientation.y
                self.camera_setpoint.rz = msg.pose.orientation.z
            else:
                rospy.logerr("Setpoint Z <<< 0")
                self.camera_setpoint.x = self.uav.pos.x
                self.camera_setpoint.y = self.uav.pos.y
                self.camera_setpoint.z = self.uav.pos.z
                self.camera_setpoint.rw = self.uav.pos.rw
                self.camera_setpoint.rx = self.uav.pos.rx
                self.camera_setpoint.ry = self.uav.pos.ry
                self.camera_setpoint.rz = self.uav.pos.rz
        else:
            rospy.logfatal("Invalid camera setpoint message type")

    def local_pos_callback(self,msg):
        self.pos.x = msg.pose.position.x
        self.pos.y = msg.pose.position.y
        self.pos.z = msg.pose.position.z
        self.pos.rw = msg.pose.orientation.w
        self.pos.rx = msg.pose.orientation.x
        self.pos.ry = msg.pose.orientation.y
        self.pos.rz = msg.pose.orientation.z


    def global_pos_callback(self,msg):
        pass
        # self.uav.global_pos.x = msg.latitude
        # self.uav.global_pos.y = msg.longitude
        # self.uav.global_pos.z = msg.altitude


    def write_serial(self,msg):
        if msg != self.prev_msg:
            # ser.write(str.encode(str(msg)+ "\n"))
            self.prev_msg = msg
            time.sleep(0.075) # Needed for ESP32C3 to read consecutive serial commands


    def quit(self):
        print("Killing node")
        self.write_serial("disarmed")
        # ser.close()
        rospy.signal_shutdown("Node shutting down")


if __name__ == '__main__':
    
    rospy.init_node('Offboard_Waypoint_Drop_Node')

    node = offboard_node()

    rospy.spin()