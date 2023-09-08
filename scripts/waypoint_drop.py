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
from scipy.spatial.transform import Rotation
import numpy as np

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
payload_drop_height=0.5

# Camera tf frames for desired setpoint
camera_frame_id="pole"
base_frame_id="base_link"
world_frame_id="map"
target_frame_id="body_setpoint"

# Threshold for jogging, when setpoint is under these conditions, drone will jog instead
threshold_jog=0.1 #m
threshold_jog_deg=10.0 #deg
max_deployment_times = 1

# ser = serial.Serial('/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:CF:79:02:99:0C-if00', 115200) #ls /dev/serial/by-id/*
# ser = serial.Serial('/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_F4:12:FA:D8:DA:58-if00', 115200) #ls /dev/serial/by-id/*

class offboard_node():

    def __init__(self):
        print("Initalising Offboard Waypoint Drop Node")

        self.uav = uav(survey_array=coordinates.waypoint_array) # Initalise UAV object
        self.uav.init_controller("close",1,0.125,1,0.125,1,0.8,0.5,0.0625) # Initalise additional controllers
        self.camera_setpoint = uav_variables() # Initalise a set of variables to store camera setpoints

        self.camera_to_body = uav_variables()
        self.prev_camera_to_body = uav_variables()

        self.setpoint_latitude=coordinates.latitude
        self.setpoint_longitude=coordinates.longitude
        # self.setpoint_altitude=coordinates.altitude

        print("Using TF Transforms for setpoints")
        # self.listener = tf.TransformListener()

        self.tfBuffer_worldtotarget = tf2_ros.Buffer()
        self.listener_worldtotarget = tf2_ros.TransformListener(self.tfBuffer_worldtotarget)
        self.tfBuffer_cameratotarget = tf2_ros.Buffer()
        self.listener_cameratotarget = tf2_ros.TransformListener(self.tfBuffer_cameratotarget)


        camera_setpoint_broadcaster = tf2_ros.TransformBroadcaster()

        self.prev_msg=""

        self.reset_timer=time.time()
        self.reset_dur=1
        self.stage="survey"
        deployment_times = 0
        self.detected = False

        self.median_height = 3.2 # default height

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)


        while not rospy.is_shutdown():

            '''
            Constantly poll to see if transform is found to object and align to it thereafter
            If transform from camera to body has changed, update the entire setpoint and cache it
            This is required due to the two different update rates of the camera to uav and the uav to global frames
            tf2 also stores a snapshot for every transform for up to 10s (http://wiki.ros.org/tf2/Tutorials/tf2%20and%20time%20%28C%2B%2B%29)
            which means that performing the whole transform will result in an outdated object if the uav is moving
            Side note: last parameter for lookuptransform is an optional blocking timeout https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp.html
            '''
            try:
                transform_camera_to_body = self.tfBuffer_cameratotarget.lookup_transform(base_frame_id, target_frame_id, rospy.Time(0))
                self.camera_to_body.save_tf2(transform_camera_to_body)
                # transform_stamped = self.tfBuffer.lookup_transform(world_frame_id, "body_setpoint", rospy.Time(0))
                # (trans,rot)=self.listener.lookupTransform(camera_frame_id, base_frame_id, rospy.Time(0))
                rospy.loginfo_once("Detected Transform from camera")
                # Check if the previous camera to body transformation has changed
                if self.camera_to_body.x != self.prev_camera_to_body.x or self.camera_to_body.y != self.prev_camera_to_body.y or self.camera_to_body.z != self.prev_camera_to_body.z or self.camera_to_body.rw != self.prev_camera_to_body.rw:

                    try:
                        # Find global to local transformation and perform transformation to mavros local frame
                        transform_stamped = self.tfBuffer_worldtotarget.lookup_transform(world_frame_id, target_frame_id, rospy.Time(0))
                        self.camera_setpoint.x = self.uav.pos.x-transform_stamped.transform.translation.x
                        self.camera_setpoint.y = self.uav.pos.y-transform_stamped.transform.translation.y
                        q1=[transform_stamped.transform.rotation.x.transform_stamped.transform.rotation.y,transform_stamped.transform.rotation.z,transform_stamped.transform.rotation.w]
                        q2=[self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz,self.uav.pos.rw]
                        q3 = Rotation.from_quat(q1).mul(Rotation.from_quat(q2)).as_quat()
                        self.camera_setpoint.rx = q3[0]
                        self.camera_setpoint.ry = q3[1]
                        self.camera_setpoint.rz = q3[2]
                        self.camera_setpoint.rw = q3[3]
                        # self.camera_setpoint.rx = transform_stamped.transform.rotation.x*self.uav.pos.rx
                        # self.camera_setpoint.ry = transform_stamped.transform.rotation.y*self.uav.pos.ry
                        # self.camera_setpoint.rz = transform_stamped.transform.rotation.z*self.uav.pos.rz
                        # self.camera_setpoint.rw = transform_stamped.transform.rotation.w*self.uav.pos.rw
                        self.detected = True

                        if self.camera_to_body.z < 0 or self.camera_to_body.z > 1.5:
                            self.camera_setpoint.z = self.uav.pos.z # Reject any outlier readings
                        else:
                            z = self.uav.pos.z+transform_stamped.transform.translation.z- payload_drop_height
                            self.median_height += 0.2 * np.sgn(z - self.median_height)
                            self.camera_setpoint.z = self.median_height

                        print(self.camera_setpoint.x,self.camera_setpoint.y,self.camera_setpoint.z)

                        self.prev_camera_to_body.save_tf2(transform_camera_to_body) #Update prior transformation

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.logdebug("Missing body setpoint tf transform")


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
            camera_setpoint_broadcaster.sendTransform(self.final_transform) #TODO TEST

            current_yaw=euler.quat2euler([self.uav.pos.rw,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz])[2] #wxyz default
            setpoint_yaw=euler.quat2euler([self.camera_setpoint.rw,self.camera_setpoint.rx,self.camera_setpoint.ry,self.camera_setpoint.rz])[2] #wxyz default


            # # No setpoint sent yet
            # if self.camera_setpoint.x == 0 and self.camera_setpoint.y ==0 and self.camera_setpoint.z ==0:
            #     self.uav.setpoint_quat(self.uav.pos.x,self.uav.pos.y,self.uav.pos.z,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz,self.uav.pos.rw) #callback local position
            
            # Camera detected droppoint, switching from GPS to local setpoint mode
            if self.detected == True :
                rospy.logwarn_once("Detected Transform")
                
                # Align X and Y first before changing altitude due to unstable height measurements from the laser rangefinger when moving
                if abs(self.camera_setpoint.x - self.uav.pos.x) > threshold_jog and abs(self.camera_setpoint.y-self.uav.pos.y) > threshold_jog:
                    self.camera_setpoint.z=self.uav.pos.z # Override height with current altitude
                    self.uav.setpoint_controller(self.camera_setpoint,"close")

                # Next add in z movement to align altitude
                elif abs(self.camera_setpoint.z-self.uav.pos.z) > threshold_jog:
                    self.uav.setpoint_controller(self.camera_setpoint,"close")

                # Finally, if it is at drop point, drop payload, assume yaw is aligned by this time
                elif abs(self.camera_setpoint.x - self.uav.pos.x) < threshold_jog and abs(self.camera_setpoint.y-self.uav.pos.y) < threshold_jog and abs(self.camera_setpoint.z-self.uav.pos.z) < threshold_jog and degrees(abs(setpoint_yaw-current_yaw)) << threshold_jog_deg:
                    rospy.loginfo_throttle_identical(2,"UAV At Camera Setpoint[%s,%s,%s], dropping",self.camera_setpoint.x,self.camera_setpoint.y,self.camera_setpoint.z)
                    if deployment_times <max_deployment_times:
                        if (self.stage=="disarmed" or self.stage=="survey"):
                            rospy.loginfo_throttle_identical(2,"Releasing Payload")
                            self.stage="payload_drop"
                            self.write_serial(self.stage)
                            self.reset_timer=rospy.get_time()
                        if (self.stage=="payload_drop" and time.time()>=self.reset_timer+self.reset_dur):
                            rospy.loginfo_throttle_identical(2,"Disarming")
                            self.stage="disarmed"
                            self.write_serial(self.stage)
                            deployment_times +=1
                    else:
                        rospy.loginfo_once("Deployment over")
                        
                # If yaw is unaligned or the x/y is not within threshold, just send the entire setpoint command
                else:
                    rospy.loginfo_throttle_identical(2,"Detected transform, UAV moving to Camera Setpoint[%s,%s,%s] from [%s,%s,%s]",round(self.camera_setpoint.x,2),round(self.camera_setpoint.y,2),round(self.camera_setpoint.z,2),round(self.uav.pos.x,2),round(self.uav.pos.y,2),round(self.uav.pos.z,2))
                    self.uav.setpoint_controller(self.camera_setpoint,"close")
 
            # Drone not at GPS Setpoint, send global coordinates
            else:
                if self.stage=="disarmed":
                    self.uav.setpoint_global(self.uav.global_pos.x, self.uav.global_pos.y, self.uav.global_pos.z-_egm96.height(self.uav.global_pos.x, self.uav.global_pos.y)) # GPS Altitude doesnt seem to be stable, so just hover at current height (with conversion)
                    rospy.loginfo_throttle_identical(5,"Sending GPS Setpoint[%s,%s,%s]", self.uav.global_pos.x, self.uav.global_pos.y, self.uav.global_pos.z-_egm96.height(self.uav.global_pos.x, self.uav.global_pos.y))
                elif self.stage=="survey":
                    self.uav.survey()
                    rospy.loginfo_throttle_identical(5,"On GPS Survey Setpoint at [%s,%s,%s]", self.uav.global_pos.x, self.uav.global_pos.y, self.uav.global_pos.z-_egm96.height(self.uav.global_pos.x, self.uav.global_pos.y))

            self.rosrate.sleep()


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
