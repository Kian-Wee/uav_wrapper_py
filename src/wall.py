#!/usr/bin/env python

import rospy
from uav import uav, uav_variables
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import tf
from math import degrees
from transforms3d import _gohlketransforms,euler
import serial
from sensor_msgs.msg import Range
import time
import numpy as np
from tf_publisher import tf_publisher
import tf2_ros

# Example code for a multi-staged multi-controller wall approach, to get it close to a wall and then slowly jog in 


rate = 100 # Update rate

# For alignment of camera_frame to drone_frame(CG), in m
cameratobody_dist = 0.5 # used for range sensor, +ve is forward
contact_threshold = 0.01 # UAV is assumed to be touching the wall at this distance

# Camera Topic for desired setpoint
camera_setpoint_topic="tf"
camera_frame_id="camera"
uav_frame_id="moose"
world_frame_id="map"

# Threshold for jogging, when setpoint is under these conditions, drone will jog instead
threshold_jog=0.5 #m
threshold_jog_deg=5 #deg  
# Rear Thruster Topic
thruster_output_topic="/thruster/pwm"
max_deployment_times = 1
hover_height=1.45

ser = serial.Serial('/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:CF:79:02:98:E4-if00', 115200) #ls /dev/serial/by-id/*

class offboard_node():

    def __init__(self):
        print("Initalising Offboard Wall Node")

        self.uav = uav() # Initalise UAV object
        self.uav.init_controller("far",1,0.125,1,0.125,1,0.8,0.5,0.0625) # Initalise additional controllers
        self.uav.init_controller("close",0.5,0.125,0.5,0.125,0.5,0.8,0.15,0.0625)
        # aux_kp=0.2
        # self.uav.init_controller("aux",aux_kp,0)
        self.camera_setpoint = uav_variables() # Initalise a set of variables to store camera setpoints

        print("Using TF Transforms for setpoints")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.final_setpoint_broadcaster = tf2_ros.TransformBroadcaster()

        self.last_acceptable_setpoint = uav_variables()
        self.last_acceptable_setpoint = self.camera_setpoint
        
        rospy.Subscriber('Range_to_wall',Range,self.range_callback)
        self.wall_dist=999
        self.wall_timer=999999999999999999999999999999
        self.wall_dur=8 #s
        self.servo_timer=999999999999999999999999999999
        self.servo_dur=2 #s
        self.adh_timer=999999999999999999999999999999
        self.adh_dur=15 #s
        self.reset_timer=999999999999999999999999999999
        self.reset_dur=1
        
        self.stage="disarmed"
        self.prev_msg=""
        self.write_serial(self.stage)

        deployment_times = 0

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)
    
        while not rospy.is_shutdown():

            
            try:
                # TODO Sending a tf transform and looking it up doesn't work in the same node for some reason
                # Look at the final transform to body_setpoint for final setpoint
                transform_stamped = self.tfBuffer.lookup_transform(world_frame_id, "body_setpoint", rospy.Time(0))
                self.camera_setpoint.x = transform_stamped.transform.translation.x
                self.camera_setpoint.y = transform_stamped.transform.translation.y
                self.camera_setpoint.z = hover_height
                self.camera_setpoint.rx = transform_stamped.transform.rotation.x
                self.camera_setpoint.ry = transform_stamped.transform.rotation.y
                self.camera_setpoint.rz = transform_stamped.transform.rotation.z
                self.camera_setpoint.rw = transform_stamped.transform.rotation.w
                # print(self.camera_setpoint.rx,self.camera_setpoint.ry,self.camera_setpoint.rz,self.camera_setpoint.rw)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logdebug("Missing body setpoint tf transform")


            current_yaw=euler.quat2euler([self.uav.pos.rw,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz])[2] #wxyz default
            setpoint_yaw=euler.quat2euler([self.camera_setpoint.rw,self.camera_setpoint.rx,self.camera_setpoint.ry,self.camera_setpoint.rz])[2] #wxyz default


            # Callback the local uav pos as a setpoint when needed to tell the uav to hover on the spot at a certain height
            self.uav_pos_setpoint=uav_variables()
            self.uav_pos_setpoint.x=self.uav.pos.x #Stop accepting last camera setpoint and commit to hovering while using only the rear thruster
            self.uav_pos_setpoint.y=self.uav.pos.y
            self.uav_pos_setpoint.z=hover_height
            self.uav_pos_setpoint.rx=self.uav.pos.rx
            self.uav_pos_setpoint.ry=self.uav.pos.ry
            self.uav_pos_setpoint.rz=self.uav.pos.rz
            self.uav_pos_setpoint.rw=self.uav.pos.rw


            # No setpoint sent yet
            if self.camera_setpoint.x == 0 and self.camera_setpoint.y ==0 and self.camera_setpoint.z ==0:
                rospy.loginfo_throttle_identical(2, "Missing setpoint/tf, hovering at current location")
                self.uav.setpoint_quat(self.uav.pos.x,self.uav.pos.y,self.uav.pos.z,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz,self.uav.pos.rw) #callback local position
                self.send_tf(self.uav.pos.x,self.uav.pos.y,self.uav.pos.z,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz,self.uav.pos.rw)
            # Wait for UAV to get to altitude first
            elif self.stage == "disarmed":
                self.uav.setpoint_controller(self.uav_pos_setpoint,"far") # Ascent to altitude at current position
                if abs(self.camera_setpoint.z-self.uav.pos.z) < threshold_jog/2:
                    self.stage = "hovering"
            elif abs(self.camera_setpoint.x - self.uav.pos.x) < threshold_jog and abs(self.camera_setpoint.y-self.uav.pos.y) < threshold_jog:  #and abs(self.camera_setpoint.z-self.uav.pos.z) < threshold_jog
                rospy.loginfo_throttle_identical(3,"Setpoint[%s,%s,%s] close to drone, jogging it inwards based on past position",self.last_acceptable_setpoint.x,self.last_acceptable_setpoint.y,self.last_acceptable_setpoint.z)
                 # Stop and yaw on the spot with less aggressive nearfield controller when close to wall
                if degrees(abs(setpoint_yaw-current_yaw)) > threshold_jog_deg:
                    rospy.loginfo_throttle_identical(1,"Yawing towards setpoint, [%s] degrees away",degrees(abs(setpoint_yaw-current_yaw)))
                    self.yaw_setpoint=uav_variables()
                    self.yaw_setpoint.x=self.uav.pos.x
                    self.yaw_setpoint.y=self.uav.pos.y
                    self.yaw_setpoint.z=hover_height
                    self.yaw_setpoint.rx=self.camera_setpoint.rx
                    self.yaw_setpoint.ry=self.camera_setpoint.ry
                    self.yaw_setpoint.rz=self.camera_setpoint.rz
                    self.yaw_setpoint.rw=self.camera_setpoint.rw
                    self.uav.setpoint_controller(self.yaw_setpoint,"close")
                    self.send_tf(self.yaw_setpoint.x,self.yaw_setpoint.y,self.yaw_setpoint.z,self.yaw_setpoint.rx,self.yaw_setpoint.ry,self.yaw_setpoint.rz,self.yaw_setpoint.rw)
               # Switch to less aggressive nearfield controller when close to wall and start translating
                elif deployment_times <max_deployment_times:


                    # for i in self.uav.controller_array:
                    #     if i.name == "aux":
                    #         thr_val = i.custom_single_controller(self.wall_dist,self.wall_dist)[0]
                    norm_thrust = round(((1 - (round(self.wall_dist,2))/(0.5)) * 70)/10)*10 #Scale rear thrust by wall distance from 0 to 0.5m from 0% thrust to 50% thrust
                    

                    if self.wall_dist <= contact_threshold and self.stage=="hovering":
                        rospy.loginfo_throttle_identical(1,"Approached wall, stabilising")
                        self.stage= "contact"
                        self.write_serial(self.stage)
                        norm_thrust = 100 #Scale rear thrust by wall distance from 0 to 0.5m from 0% thrust to 100% thrust
                        
                        self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                        self.wall_timer=rospy.get_time()

                    elif (self.wall_dist <= contact_threshold and self.stage=="contact" and time.time()>=self.wall_timer+self.wall_dur):
                        rospy.loginfo_throttle_identical(1,"Touched wall and stabilised, releasing adhesive")
                        self.stage= "glue_release"
                        self.write_serial(self.stage)
                        norm_thrust = 100 #Scale rear thrust by wall distance from 0 to 0.5m from 0% thrust to 100% thrust

                        self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                        self.servo_timer=rospy.get_time()

                    elif (self.wall_dist <= contact_threshold and self.stage=="glue_release" and time.time()>=self.servo_timer+self.servo_dur):
                        rospy.loginfo_throttle_identical(0.5,"Adhesive Released, turning on LED")
                        self.stage= "uv_on"
                        self.write_serial(self.stage)
                        norm_thrust = 100 #Scale rear thrust by wall distance from 0 to 0.5m from 0% thrust to 100% thrust

                        self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                        self.adh_timer=rospy.get_time()
                        
                    elif (self.wall_dist <= contact_threshold and self.stage=="uv_on" and time.time()>=self.adh_timer+self.adh_dur):
                        rospy.loginfo_throttle_identical(1,"Dropping payload")
                        self.stage="payload_drop"
                        self.write_serial(self.stage)
                        norm_thrust = 100 #Scale rear thrust by wall distance from 0 to 0.5m from 0% thrust to 100% thrust

                        self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                        self.reset_timer=rospy.get_time()

                    elif (self.wall_dist <= contact_threshold and self.stage=="payload_drop" and time.time()>=self.reset_timer+self.reset_dur):
                        rospy.loginfo_throttle_identical(0.5,"Disarming")
                        self.stage="uv_off"
                        self.write_serial(self.stage)
                        self.stage="payload_reset"
                        self.write_serial(self.stage)
                        self.write_serial('0') # Set thruster to 0
                        norm_thrust = 0
                        self.stage="disarmed"
                        self.write_serial(self.stage)
                        deployment_times +=1

                        self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                    # In between deployment stages, it gets told to hover at the current position
                    else:

                        self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                    rospy.loginfo_throttle_identical(2,"<--------Yaw within margin. Wall @ [%s], Moving with rear thruster @ [%s]", self.wall_dist, norm_thrust)
                    self.write_serial(norm_thrust)
                else:
                    rospy.loginfo_once("Deployment over")
                    self.uav.setpoint_controller(self.last_acceptable_setpoint,"close") # Stop reading new setpoints and cache the setpoint
                    self.send_tf(self.last_acceptable_setpoint.x,self.last_acceptable_setpoint.y,self.last_acceptable_setpoint.z,self.last_acceptable_setpoint.rx,self.last_acceptable_setpoint.ry,self.last_acceptable_setpoint.rz,self.last_acceptable_setpoint.rw)
                    

            # Approach setpoint with an aggressive controller when far 
            else:
                rospy.loginfo_throttle_identical(2,"Setpoint far from drone, using controller %s",self.uav.pos.x)
                self.last_acceptable_setpoint = self.camera_setpoint
                self.uav.setpoint_controller(self.camera_setpoint,"far")
                self.send_tf(self.camera_setpoint.x,self.camera_setpoint.y,self.camera_setpoint.z,self.camera_setpoint.rx,self.camera_setpoint.ry,self.camera_setpoint.rz,self.camera_setpoint.rw)

            
            self.rosrate.sleep()


    def range_callback(self, msg):
        self.wall_dist = msg.range - cameratobody_dist
        # print(self.wall_dist)


    def send_tf(self, x, y, z, rx, ry, rz, rw):
        self.final_transform= tf2_ros.TransformStamped()
        self.final_transform.header.frame_id = world_frame_id
        self.final_transform.header.stamp = rospy.get_rostime()
        self.final_transform.child_frame_id = "final_setpoint"

        # Send out transform again for visualisation
        self.final_transform.transform.translation.x = x
        self.final_transform.transform.translation.y = y
        self.final_transform.transform.translation.z = z
        self.final_transform.transform.rotation.x  = rx
        self.final_transform.transform.rotation.y  = ry
        self.final_transform.transform.rotation.z  = rz
        self.final_transform.transform.rotation.w  = rw

        self.final_setpoint_broadcaster.sendTransform(self.final_transform)


    def write_serial(self,msg):
        if msg != self.prev_msg:
            ser.write(str.encode(str(msg)+ "\n"))
            self.prev_msg = msg
            time.sleep(0.075) # Needed for ESP32C3 to read consecutive serial commands

    
    def quit(self):
        print("Killing node")
        ser.write(str.encode('disarmed' + "\n"))
        ser.close()
        rospy.signal_shutdown("Node shutting down")


if __name__ == '__main__':
    
    rospy.init_node('Offboard_Wall_Node')

    node = offboard_node()

    rospy.spin()
