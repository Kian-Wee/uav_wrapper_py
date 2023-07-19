#!/usr/bin/env python

import rospy
from uav import uav, uav_variables
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from tf2_msgs.msg import TFMessage
import tf
from math import degrees
from transforms3d import _gohlketransforms,euler
import serial
from sensor_msgs.msg import Range
import time
import coordinates

# Example code for a multi-staged multi-controller wall approach, to get it close to a wall and then slowly jog in 

rate = 60 # Update rate

# For alignment of camera_frame to drone_frame(CG), in m
cameratobody_x = 0 # +ve is forward

# Camera Topic for desired setpoint
camera_setpoint_topic="/tf"
camera_frame_id="/camera"
world_frame_id="/map"

# Threshold for jogging, when setpoint is under these conditions, drone will jog instead
threshold_jog=0.1 #m
threshold_jog_deg=10 #deg
max_deployment_times = 1


ser = serial.Serial('/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_F4:12:FA:D8:DA:58-if00', 115200) #ls /dev/serial/by-id/*

class offboard_node():

    def __init__(self):
        print("Initalising Offboard Waypoint Drop Node")

        self.uav = uav() # Initalise UAV object
        self.uav.init_controller("far",0.5,0.125,0.5,0.125,0.5,0.8,0.25,0.0625) # Initalise additional controllers
        self.camera_setpoint = uav_variables() # Initalise a set of variables to store camera setpoints

        self.latitude=coordinates.latitude
        self.longitude=coordinates.longitude
        self.altitude=coordinates.altitude

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

        self.reset_timer=time.time()
        self.reset_dur=1
        self.release_stage="disarmed"

        deployment_times = 0
        self.detected = False

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)

        self.global_setpoint_publisher = rospy.Publisher("/mavros/setpoint_position/global", GeoPoseStamped, queue_size=1)
    
        while not rospy.is_shutdown():

            try:
                (trans,rot)=self.listener.lookupTransform(camera_frame_id, world_frame_id, rospy.Time(0))
                self.camera_setpoint.x = trans[0]+self.uav.pos.x
                self.camera_setpoint.y = trans[1]+self.uav.pos.y
                self.camera_setpoint.z = trans[2]+self.uav.pos.z
                self.camera_setpoint.rx = rot[0]*self.uav.pos.rx
                self.camera_setpoint.ry = rot[1]*self.uav.pos.ry
                self.camera_setpoint.rz = rot[2]*self.uav.pos.rz
                self.camera_setpoint.rw = rot[3]*self.uav.pos.rw
                self.detected = True
                rospy.loginfo_once("Detected Transform from camera")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logdebug("Missing tf transform")

            current_yaw=euler.quat2euler([self.uav.pos.rw,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz])[2] #wxyz default
            setpoint_yaw=euler.quat2euler([self.camera_setpoint.rw,self.camera_setpoint.rx,self.camera_setpoint.ry,self.camera_setpoint.rz])[2] #wxyz default

            # No setpoint sent yet
            if self.camera_setpoint.x == 0 and self.camera_setpoint.y ==0 and self.camera_setpoint.z ==0:
                self.uav.setpoint_quat(self.uav.pos.x,self.uav.pos.y,self.uav.pos.z,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz,self.uav.pos.rw) #callback local position
            
            # Camera detected droppoint, switching from GPS to local setpoint mode
            elif self.detected == True :
                self.uav.setpoint_controller(self.camera_setpoint,"far")
                # At drop point, dropping payload
                if abs(self.camera_setpoint.x - self.uav.pos.x) < threshold_jog and abs(self.camera_setpoint.y-self.uav.pos.y) < threshold_jog and abs(self.camera_setpoint.z-self.uav.pos.z) < threshold_jog and degrees(abs(setpoint_yaw-current_yaw)) << threshold_jog_deg:
                    rospy.loginfo_once("UAV At Camera Setpoint[%s,%s,%s], dropping",self.camera_setpoint.x,self.camera_setpoint.y,self.camera_setpoint.z)
                    if deployment_times <max_deployment_times:
                        if (self.release_stage=="disarmed"):
                            rospy.loginfo_once("Dropping payload")
                            self.release_stage="payload_drop"
                            ser.write(self.release_stage)
                            self.reset_timer=rospy.get_time()
                        if (self.release_stage=="payload_drop" and time.time()>=self.reset_timer+self.reset_dur):
                            rospy.loginfo_once("Disarming")
                            self.release_stage="payload_reset"
                            ser.write(self.release_stage)
                            ser.write("0")
                            self.release_stage="disarmed"
                            deployment_times +=1
                    else:
                        rospy.loginfo_once("Deployment over")

            # Drone not at GPS Setpoint, send global coordinates
            else:
                msg=GeoPoseStamped()
                msg.pose.position.latitude=self.latitude
                msg.pose.position.longitude=self.longitude
                msg.pose.position.altitude=self.altitude
                self.global_setpoint_publisher.publish(msg)

            self.rosrate.sleep()


    def camera_listener_callback(self, msg):
        rospy.loginfo("New Camera setpoint(x:"+str(msg.pose.position.x)+", y:"+str(msg.pose.position.y)+", z:"+str(msg.pose.position.z)+")")
                # If TF is used as the as the position
        # if msg._type=="tf2_msgs/TFMessage":
        #     print(msg.transforms[0].header.frame_id,msg.transforms[0].child_frame_id)
        #     if msg.transforms[0].header.frame_id == world_frame_id and msg.transforms[0].child_frame_id == camera_frame_id:
        #         self.camera_setpoint.x = msg.transforms[0].transform.translation.x
        #         self.camera_setpoint.y = msg.transforms[0].transform.translation.y
        #         self.camera_setpoint.z = msg.transforms[0].transform.translation.z
        #         self.camera_setpoint.rx = msg.transforms[0].transform.rotation.x
        #         self.camera_setpoint.ry = msg.transforms[0].transform.rotation.y
        #         self.camera_setpoint.rz = msg.transforms[0].transform.rotation.z
        #         self.camera_setpoint.rw = msg.transforms[0].transform.rotation.w
        if msg._type=="geometry_msgs/PoseStamped":
            if self.camera_setpoint.z > 0:
                self.camera_setpoint.x = msg.pose.position.x
                self.camera_setpoint.y = msg.pose.position.y
                self.camera_setpoint.z = msg.pose.position.z
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


    def quit(self):
        print("Killing node")
        ser.write('D0')
        ser.close()
        rospy.signal_shutdown("Node shutting down")


if __name__ == '__main__':
    
    rospy.init_node('Offboard_Waypoint_Drop_Node')

    node = offboard_node()

    rospy.spin()