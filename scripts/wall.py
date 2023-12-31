#!/usr/bin/env python3

import rospy
from uav import uav, uav_variables
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32
import tf
from math import degrees
from transforms3d import _gohlketransforms,euler
from mavros_msgs.msg import State
import serial
from sensor_msgs.msg import Range
import time
import numpy as np
from tf_publisher import tf_publisher
import tf2_ros

# Example code for a multi-staged multi-controller wall approach, to get it close to a wall and then slowly jog in 


rate = 60 # Update rate

# For alignment of camera_frame to drone_frame(CG), in m
cameratobody_dist = 0.5 # used for range sensor, +ve is forward
contact_threshold = 0.05 # UAV is assumed to be touching the wall at this distance
wall_threshold = 0.98 # Will only proceed with deployment if probability is equals or higher than this threshold

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
max_thrust=100
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


        print("Using TF Transforms for setpoints")

        self.tfBuffer_target = tf2_ros.Buffer()
        self.listener_target = tf2_ros.TransformListener(self.tfBuffer_target)
        self.camera_setpoint = uav_variables() # Custom class to store pose and orientation

        self.tfBuffer_traverse = tf2_ros.Buffer()
        self.listener_traverse = tf2_ros.TransformListener(self.tfBuffer_traverse)
        self.traverse_setpoint = uav_variables()

        self.tfBuffer_yaw = tf2_ros.Buffer()
        self.listener_yaw = tf2_ros.TransformListener(self.tfBuffer_yaw)
        self.yaw_setpoint = uav_variables()

        self.final_setpoint_broadcaster = tf2_ros.TransformBroadcaster()

        self.last_acceptable_setpoint = uav_variables()
        self.last_acceptable_setpoint = self.camera_setpoint
        
        rospy.Subscriber('Range_to_wall',Range,self.range_callback)
        self.wall_dist=999
        self.wall_timer=999999999999999999999999999999
        self.wall_dur=8 #s
        self.yaw_halt_timer=999999999999999999999999999999
        self.yaw_halt_dur=1
        self.check_halt_timer=999999999999999999999999999999
        self.check_halt_dur=1.5
        self.spread_timer=999999999999999999999999999999
        self.spread_dur=1.5 #s
        self.adh_timer=999999999999999999999999999999
        self.adh_dur=10 #s
        self.reset_timer=999999999999999999999999999999
        self.reset_dur=1

        rospy.Subscriber('/GR_Ratio',Float32,self.wall_prob_callback)
        rospy.Subscriber('/mavros/state',State,self.mavros_state_callback)
        
        self.stage="disarmed"
        self.prev_msg=""
        self.write_serial(self.stage)

        deployment_times = 0

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)
  
    
        while not rospy.is_shutdown():

            # TODO Sending a tf transform and looking it up doesn't work in the same node for some reason
            # Look at the final transform to body_setpoint for final setpoint
            try:
                transform_stamped = self.tfBuffer_target.lookup_transform(world_frame_id, "body_setpoint", rospy.Time(0))
                self.camera_setpoint.update(x = transform_stamped.transform.translation.x,y = transform_stamped.transform.translation.y,z = hover_height,
                                            rx = transform_stamped.transform.rotation.x,ry = transform_stamped.transform.rotation.y,rz = transform_stamped.transform.rotation.z,rw = transform_stamped.transform.rotation.w)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logdebug("Missing body setpoint tf transform")

            try:
                transform_stamped2 = self.tfBuffer_traverse.lookup_transform(world_frame_id, "traverse_pt", rospy.Time(0))
                self.traverse_setpoint.update(x = transform_stamped2.transform.translation.x,y = transform_stamped2.transform.translation.y,z = hover_height,
                                            rx = transform_stamped2.transform.rotation.x,ry = transform_stamped2.transform.rotation.y,rz = transform_stamped2.transform.rotation.z,rw = transform_stamped2.transform.rotation.w)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logdebug("Missing body setpoint tf transform")

            try:
                transform_stamped3 = self.tfBuffer_yaw.lookup_transform(world_frame_id, "yaw_pt", rospy.Time(0))
                self.yaw_setpoint.update(x = transform_stamped3.transform.translation.x,y = transform_stamped3.transform.translation.y,z = hover_height,
                                            rx = transform_stamped3.transform.rotation.x,ry = transform_stamped3.transform.rotation.y,rz = transform_stamped3.transform.rotation.z,rw = transform_stamped3.transform.rotation.w)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logdebug("Missing body setpoint tf transform")



            current_yaw=euler.quat2euler([self.uav.pos.rw,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz])[2] #wxyz default
            setpoint_yaw=euler.quat2euler([self.camera_setpoint.rw,self.camera_setpoint.rx,self.camera_setpoint.ry,self.camera_setpoint.rz])[2] #wxyz default


            # Callback the local uav pos as a setpoint when needed to tell the uav to hover on the spot at a certain height
            self.uav_pos_setpoint=uav_variables(x=self.uav.pos.x,y=self.uav.pos.y,z=hover_height,rx=self.uav.pos.rx,ry=self.uav.pos.ry,rz=self.uav.pos.rz,rw=self.uav.pos.rw)


            # Proceed with deployment if max number of deployment has not been reached
            if deployment_times <max_deployment_times:

                # Wait for UAV to get to altitude first
                if self.stage == "disarmed":
                    rospy.loginfo_throttle_identical(5, "Moving to set altitude")
                    self.uav.setpoint_controller(self.uav_pos_setpoint,"far") # Ascent to altitude at current position
                    self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)
                    if abs(hover_height-self.uav.pos.z) < threshold_jog/2:
                        self.stage = "hovering"
                        rospy.logwarn_throttle_identical(2, "At hover height")

                # If no setpoint is seen, yaw left until setpoint is seen/issued
                elif self.camera_setpoint.x == 0 and self.camera_setpoint.y ==0 and self.camera_setpoint.z ==0:
                    rospy.loginfo_throttle_identical(2, "Missing setpoint/tf, yawing 2deg left until wall detected")
                    self.uav.setpoint_quat(self.yaw_setpoint.x,self.yaw_setpoint.y,self.yaw_setpoint.z,self.yaw_setpoint.rx,self.yaw_setpoint.ry,self.yaw_setpoint.rz,self.yaw_setpoint.rw) #callback local position
                    self.send_tf(self.yaw_setpoint.x,self.yaw_setpoint.y,self.yaw_setpoint.z,self.yaw_setpoint.rx,self.yaw_setpoint.ry,self.yaw_setpoint.rz,self.yaw_setpoint.rw)

                # Hover and yaw first
                elif self.stage=="hovering":

                    # Stop and yaw on the spot until aligned
                    if degrees(abs(setpoint_yaw-current_yaw)) > threshold_jog_deg:
                        rospy.loginfo_throttle_identical(1,"Yawing towards setpoint, [%s] degrees away",degrees(abs(setpoint_yaw-current_yaw)))
                        self.yaw_setpoint=uav_variables(x=self.uav.pos.x,y=self.uav.pos.y,z=hover_height,rx=self.camera_setpoint.rx,ry=self.camera_setpoint.ry,rz=self.camera_setpoint.rz,rw=self.camera_setpoint.rw)
                        self.uav.setpoint_controller(self.yaw_setpoint,"close") # To allow it to yaw slower
                        self.send_tf(self.yaw_setpoint.x,self.yaw_setpoint.y,self.yaw_setpoint.z,self.yaw_setpoint.rx,self.yaw_setpoint.ry,self.yaw_setpoint.rz,self.yaw_setpoint.rw)
                    else:
                        rospy.logwarn_throttle_identical(1,"Finished yawing")
                        self.stage="yaw-halt"
                        self.uav.setpoint_controller(self.yaw_setpoint,"close") # To allow it to yaw slower
                        self.send_tf(self.yaw_setpoint.x,self.yaw_setpoint.y,self.yaw_setpoint.z,self.yaw_setpoint.rx,self.yaw_setpoint.ry,self.yaw_setpoint.rz,self.yaw_setpoint.rw)
                        self.yaw_halt_timer=time.time()
                
                elif self.stage=="yaw-halt":
                    rospy.loginfo_throttle_identical(1,"Halting after yaw")
                    if time.time() >= self.yaw_halt_dur + self.yaw_halt_timer:
                        self.stage="yaw"
                        rospy.logwarn_throttle_identical(1,"Finish halting at deployable surface")
                    self.uav.setpoint_controller(self.uav_pos_setpoint,"far")
                    self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                # Check if surface can be deployed on, else freezes here
                elif self.stage=="yaw": # and self.mavros_state=="OFFBOARD" TODO
                    rospy.loginfo_throttle_identical(3,"Checking surface for deployability")
                    # Will only proceed to next stage if wall_threshold is met
                    if self.wall_prob >= wall_threshold:
                        self.stage="check-halt"
                        rospy.logwarn_throttle_identical(1,"Found deployable surface")
                        self.uav.setpoint_controller(self.uav_pos_setpoint,"far")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)
                        self.check_halt_timer=time.time()
                    else:
                        self.traverse_setpoint.y=-0.25
                        self.traverse_setpoint.rx=self.camera_setpoint.rx
                        self.traverse_setpoint.ry=self.camera_setpoint.ry
                        self.traverse_setpoint.rz=self.camera_setpoint.rz
                        self.traverse_setpoint.rw=self.camera_setpoint.rw
                        self.uav.setpoint_controller(self.traverse_setpoint,"far")
                        self.send_tf(self.traverse_setpoint.x,self.traverse_setpoint.y,self.traverse_setpoint.z,self.traverse_setpoint.rx,self.traverse_setpoint.ry,self.traverse_setpoint.rz,self.traverse_setpoint.rw)
                        rospy.logwarn_throttle_identical(2,"Traversing left to deployable surface")

                elif self.stage=="check-halt":
                    rospy.loginfo_throttle_identical(2,"Halting at deployable surface")
                    if time.time() >= self.check_halt_dur + self.check_halt_timer:
                        self.stage="check"
                        rospy.logwarn_throttle_identical(1,"Finish halting at deployable surface")
                    self.uav.setpoint_controller(self.uav_pos_setpoint,"far")
                    self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                # When close to setpoint, use different controllers and start the release sequence when conditions are met
                elif abs(self.camera_setpoint.x - self.uav.pos.x) <= threshold_jog and abs(self.camera_setpoint.y-self.uav.pos.y) <= threshold_jog:  #and abs(self.camera_setpoint.z-self.uav.pos.z) < threshold_jog
                    # Contacted wall, max thrusters and hover at current spot
                    if self.wall_dist <= contact_threshold and self.stage=="check":
                        rospy.logwarn_throttle_identical(1,"Approached wall, stabilising")
                        self.stage= "contact"
                        self.write_serial(self.stage)
                        norm_thrust = max_thrust
                        
                        self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                        self.wall_timer=time.time()

                    elif (self.wall_dist <= contact_threshold and self.stage=="contact" and time.time()>=self.wall_timer+self.wall_dur):
                        rospy.logwarn_throttle_identical(1,"Touched wall and stabilised, releasing adhesive")
                        self.stage= "glue_release"
                        self.write_serial(self.stage)
                        norm_thrust = max_thrust

                        self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                        self.spread_timer=time.time()

                    elif (self.wall_dist <= contact_threshold and self.stage=="glue_release" and time.time()>=self.spread_timer+self.spread_dur):
                        rospy.logwarn_throttle_identical(1,"Adhesive Released, turning on LED")
                        self.stage= "glue_reset"
                        self.write_serial(self.stage)
                        self.stage= "uv_on"
                        self.write_serial(self.stage)
                        norm_thrust = max_thrust
                        self.write_serial(str(norm_thrust)) # Set thruster to 0

                        self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                        self.adh_timer=time.time()
                        

                    elif (self.wall_dist <= contact_threshold and self.stage=="uv_on" and time.time()>=self.adh_timer+self.adh_dur):
                        rospy.logwarn_throttle_identical(1,"Dropping payload")
                        self.stage= "uv_off"
                        self.write_serial(self.stage)
                        norm_thrust = max_thrust
                        self.write_serial(str(norm_thrust))
                        self.stage="payload_drop"
                        self.write_serial(self.stage)

                        self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                        self.reset_timer=time.time()

                    elif (self.wall_dist <= contact_threshold and self.stage=="payload_drop" and time.time()>=self.reset_timer+self.reset_dur):
                        rospy.logwarn_throttle_identical(1,"Disarming")
                        self.stage="payload_reset"
                        self.write_serial(self.stage)
                        norm_thrust = 0
                        self.write_serial(str(norm_thrust)) # Set thruster to 0
                        self.stage="disarmed"
                        self.write_serial(self.stage)
                        deployment_times +=1

                        self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                        self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                    # In between deployment stages, it comes to this else purgatory
                    else:
                        # If not contacted wall yet, scale up thrust linearly
                        if self.wall_dist > contact_threshold:
                            rospy.logwarn_throttle_identical(1,"Wall %sm away",self.wall_dist)
                            norm_thrust = round(((1 - (round(self.wall_dist,2))/(0.5)) * max_thrust)/10)*10 #Scale rear thrust by wall distance from 0 to 0.5m from 0% thrust to 50% thrust
                            self.uav.setpoint_controller(self.last_acceptable_setpoint,"close") # Stop reading new setpoints and cache the setpoint
                            self.send_tf(self.last_acceptable_setpoint.x,self.last_acceptable_setpoint.y,self.last_acceptable_setpoint.z,self.last_acceptable_setpoint.rx,self.last_acceptable_setpoint.ry,self.last_acceptable_setpoint.rz,self.last_acceptable_setpoint.rw)
                        # If contacted wall, set thruster to full and hover at current position to avoid PX4 MPC overcompensating
                        else:
                            norm_thrust = max_thrust
                            self.uav.setpoint_controller(self.uav_pos_setpoint,"close")
                            self.send_tf(self.uav_pos_setpoint.x,self.uav_pos_setpoint.y,self.uav_pos_setpoint.z,self.uav_pos_setpoint.rx,self.uav_pos_setpoint.ry,self.uav_pos_setpoint.rz,self.uav_pos_setpoint.rw)

                    rospy.loginfo_throttle_identical(3,"Wall @ [%s], Moving with rear thruster @ [%s]. Setpoint[%s,%s,%s] close to drone", self.wall_dist, norm_thrust,self.last_acceptable_setpoint.x,self.last_acceptable_setpoint.y,self.last_acceptable_setpoint.z)
                    self.write_serial(norm_thrust)

                # Approach setpoint with an aggressive controller when far 
                else:
                    rospy.loginfo_throttle_identical(5,"Setpoint far from drone, using controller [%s,%s,%s]",self.camera_setpoint.x,self.camera_setpoint.y,self.camera_setpoint.z)
                    self.last_acceptable_setpoint = self.camera_setpoint
                    self.uav.setpoint_controller(self.camera_setpoint,"far")
                    self.send_tf(self.camera_setpoint.x,self.camera_setpoint.y,self.camera_setpoint.z,self.camera_setpoint.rx,self.camera_setpoint.ry,self.camera_setpoint.rz,self.camera_setpoint.rw)

                
            # No setpoint sent yet(should not be running)
            elif self.camera_setpoint.x == 0 and self.camera_setpoint.y ==0 and self.camera_setpoint.z ==0:
                rospy.loginfo_throttle_identical(2, "Missing setpoint/tf, hovering at current location")
                self.uav.setpoint_quat(self.uav.pos.x,self.uav.pos.y,self.uav.pos.z,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz,self.uav.pos.rw) #callback local position
                self.send_tf(self.uav.pos.x,self.uav.pos.y,self.uav.pos.z,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz,self.uav.pos.rw)
                
            # Deployment over, hover at current setpoint
            else:
                rospy.logwarn_once("Deployment over")
                self.uav.setpoint_controller(self.last_acceptable_setpoint,"close")
                self.send_tf(self.last_acceptable_setpoint.x,self.last_acceptable_setpoint.y,self.last_acceptable_setpoint.z,self.last_acceptable_setpoint.rx,self.last_acceptable_setpoint.ry,self.last_acceptable_setpoint.rz,self.last_acceptable_setpoint.rw)
                

            self.rosrate.sleep()


    def range_callback(self, msg):
        self.wall_dist = msg.range - cameratobody_dist


    def wall_prob_callback(self, msg):
        self.wall_prob = msg.data


    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode


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
            time.sleep(0.15) # Time needed for ESP32C3 to read consecutive serial commands (0.075-min, 0.15-conservative), commands might not activate if set too low

    
    def quit(self):
        print("Killing node")
        ser.write(str.encode('disarmed' + "\n"))
        ser.close()
        rospy.signal_shutdown("Node shutting down")


if __name__ == '__main__':
    
    rospy.init_node('Offboard_Wall_Node')

    node = offboard_node()

    rospy.spin()
