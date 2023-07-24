#!/usr/bin/env python

import rospy
from uav import uav, uav_variables
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import tf
from math import degrees
from transforms3d import _gohlketransforms,euler
import serial
import math
from sensor_msgs.msg import Range
import time

# Example code for a multi-staged multi-controller wall approach, to get it close to a wall and then slowly jog in 


rate = 60 # Update rate

# For alignment of camera_frame to drone_frame(CG), in m
cameratobody_x = 0.4 # +ve is forward

# Camera Topic for desired setpoint
camera_setpoint_topic="/tf"
camera_frame_id="/camera"
world_frame_id="/map"

# Threshold for jogging, when setpoint is under these conditions, drone will jog instead
threshold_jog=0.5 #m
threshold_jog_deg=5 #deg
# Rear Thruster Topic
thruster_output_topic="/thruster/pwm"
max_deployment_times = 1

ser = serial.Serial('/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:CF:79:02:98:E4-if00', 115200) #ls /dev/serial/by-id/*
# ser = serial.Serial('/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_F4:12:FA:D8:DA:58-if00', 115200) #ls /dev/serial/by-id/*

class offboard_node():

    def __init__(self):
        print("Initalising Offboard Wall Node")

        self.uav = uav() # Initalise UAV object
        self.uav.init_controller("far",0.5,0.125,0.5,0.125,0.5,0.8,0.25,0.0625) # Initalise additional controllers
        self.uav.init_controller("close",0.1,0.125,0.1,0.125,0.1,0.8,0.5,0.0625)
        aux_kp=0.2
        self.uav.init_controller("aux",aux_kp,0)
        self.camera_setpoint = uav_variables() # Initalise a set of variables to store camera setpoints

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
        
        print(f"Using " + camera_setpoint_topic + " setpoint topic")

        self.last_acceptable_setpoint = uav_variables()
        self.last_acceptable_setpoint = self.camera_setpoint

        rospy.Subscriber('Range_to_wall',Range,self.range_callback)
        self.wall_dist=999
        self.wall_timer=time.time()
        self.wall_dur=3 #s
        self.adh_timer=time.time()
        self.adh_dur=10 #s
        self.reset_timer=time.time()
        self.reset_dur=1
        self.release_stage="disarmed"

        deployment_times = 0

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)
    
        while not rospy.is_shutdown():

            try:
                (trans,rot)=self.listener.lookupTransform(camera_frame_id, world_frame_id, rospy.Time(0))
                self.camera_setpoint.x = trans[0]+self.uav.pos.x
                self.camera_setpoint.y = trans[1]+self.uav.pos.y
                self.camera_setpoint.z = 1.2 #trans[2]+self.uav.pos.z
                self.camera_setpoint.rx = rot[0]*self.uav.pos.rx
                self.camera_setpoint.ry = rot[1]*self.uav.pos.ry
                self.camera_setpoint.rz = rot[2]*self.uav.pos.rz
                self.camera_setpoint.rw = rot[3]*self.uav.pos.rw
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logdebug("Missing tf transform")

            current_yaw=euler.quat2euler([self.uav.pos.rw,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz])[2] #wxyz default
            setpoint_yaw=euler.quat2euler([self.camera_setpoint.rw,self.camera_setpoint.rx,self.camera_setpoint.ry,self.camera_setpoint.rz])[2] #wxyz default

            # No setpoint sent yet
            if self.camera_setpoint.x == 0 and self.camera_setpoint.y ==0 and self.camera_setpoint.z ==0:
                self.uav.setpoint_quat(self.uav.pos.x,self.uav.pos.y,self.uav.pos.z,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz,self.uav.pos.rw) #callback local position
            
            elif abs(self.camera_setpoint.x - self.uav.pos.x) < threshold_jog and abs(self.camera_setpoint.y-self.uav.pos.y) < threshold_jog and abs(self.camera_setpoint.z-self.uav.pos.z) < threshold_jog:
                rospy.loginfo_once("Setpoint[%s,%s,%s] close to drone, jogging it inwards based on past position",self.last_acceptable_setpoint.x,self.last_acceptable_setpoint.y,self.last_acceptable_setpoint.z)
                 # Stop and yaw on the spot with less agressive nearfield controller when close to wall
                if degrees(abs(setpoint_yaw-current_yaw)) > threshold_jog_deg:
                    rospy.loginfo_once("Yawing towards setpoint")
                    self.yaw_setpoint=uav_variables()
                    self.yaw_setpoint.x=self.uav.pos.x
                    self.yaw_setpoint.y=self.uav.pos.y
                    self.yaw_setpoint.z=1.2 #self.uav.pos.z
                    self.yaw_setpoint.rx=self.camera_setpoint.rx
                    self.yaw_setpoint.ry=self.camera_setpoint.ry
                    self.yaw_setpoint.rz=self.camera_setpoint.rz
                    self.yaw_setpoint.rw=self.camera_setpoint.rw
                    self.uav.setpoint_controller(self.camera_setpoint,"close")
                    print("yaw")
               # Switch to less aggressive nearfield controller when close to wall and start translating
                elif deployment_times <max_deployment_times:
                    self.uav.setpoint_controller(self.camera_setpoint,"close")
                    rospy.loginfo_once("Yaw within margin, moving towards setpoint and using rear thruster")
                    thr_val = self.uav.controller_array["aux"].custom_single_controller(self.wall_dist,self.wall_dist)
                    print(str(translate(thr_val, 0, aux_kp, 0, 100)))
                    ser.write(str(translate(thr_val, 0, aux_kp, 0, 100)))
                    if self.wall_dist <= 0.05 and self.release_stage=="disarmed":
                        rospy.loginfo_once("contact")
                        self.wall_timer=rospy.get_time()
                        self.release_stage= "Approached wall, stabalising"
                    if (self.wall_dist <= 0.05 and self.release_stage=="contact" and time.time()>=self.wall_timer+self.wall_dur):
                        rospy.loginfo_once("Touched wall and stabalised, releasing adhesive")
                        self.release_stage= "glue_release"
                        ser.write(self.release_stage)
                        self.adh_timer=rospy.get_time()
                    if (self.wall_dist <= 0.05 and self.release_stage=="glue_release" and time.time()>=self.adh_timer+self.adh_dur):
                        rospy.loginfo_once("Dropping payload")
                        self.release_stage="payload_drop"
                        ser.write(self.release_stage)
                        self.reset_timer=rospy.get_time()
                    if (self.wall_dist <= 0.05 and self.release_stage=="payload_drop" and time.time()>=self.reset_timer+self.reset_dur):
                        rospy.loginfo_once("Disarming")
                        self.release_stage="uv_off"
                        ser.write(self.release_stage)
                        self.release_stage="payload_reset"
                        ser.write(self.release_stage)
                        self.release_stage="disarmed"
                        ser.write("0")
                        deployment_times +=1
                else:
                    rospy.loginfo_once("Deployment over")

            # Approach setpoint with aggressive controller when far 
            else:
                rospy.loginfo_once("Setpoint far from drone, using controller %s",self.uav.pos.x)
                self.last_acceptable_setpoint = self.camera_setpoint
                self.uav.setpoint_controller(self.camera_setpoint,"far")

            self.rosrate.sleep()


    def range_callback(self, msg):
        self.wall_dist = msg.range - cameratobody_x


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
        ser.write(str.encode('D0'))
        ser.close()
        rospy.signal_shutdown("Node shutting down")


def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


if __name__ == '__main__':
    
    rospy.init_node('Offboard_Wall_Node')

    node = offboard_node()

    rospy.spin()