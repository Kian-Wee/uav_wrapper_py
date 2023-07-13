#!/usr/bin/env python

import rospy
from uav import uav, uav_variables
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import tf
from math import degrees
from transforms3d import _gohlketransforms,euler

# Example code for a multi-staged multi-controller wall approach, to get it close to a wall and then slowly jog in 


rate = 60 # Update rate

# For alignment of camera_frame to drone_frame(CG), in m
cameratobody_x =0 # +ve is forward
cameratobody_y =0 # +ve is left
cameratobody_z =0 # +ve is up 

# Camera Topic for desired setpoint
camera_setpoint_mode="/tf" #listen for setpoint at /tf or at /topic(given by camera_setpoint_topic)
camera_setpoint_topic="/camera_setpoint" # if tf is not in use
camera_frame_id="/camera"
world_frame_id="/map"

# Threshold for jogging, when setpoint is under these conditions, drone will jog instead
threshold_jog=0.5 #m
threshold_jog_deg=10 #deg
# Rear Thruster Topic
thruster_output_topic="/thruster/pwm"

class offboard_node():

    def __init__(self):
        print("Initalising Offboard Wall Node")

        self.uav = uav() # Initalise UAV object
        self.uav.init_controller("far",0.5,0.125,0.5,0.125,0.5,0.8,0.25,0.0625) # Initalise additional controllers
        self.uav.init_controller("close",0.1,0.125,0.1,0.125,0.1,0.8,0.5,0.0625)
        self.uav.init_controller("thruster",0.2,0)
        self.camera_setpoint = uav_variables() # Initalise a set of variables to store camera setpoints

        if camera_setpoint_mode == "/topic":
            # rospy.Subscriber(
            #     camera_setpoint_topic,
            #     PoseStamped,
            #     self.camera_listener_callback)
            rospy.Subscriber(
                camera_setpoint_topic,
                TFMessage,
                self.camera_listener_callback)
        else:
            self.listener = tf.TransformListener()

        self.last_acceptable_setpoint = uav_variables()
        self.last_acceptable_setpoint = self.camera_setpoint

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)
    
        while not rospy.is_shutdown():

            current_yaw=euler.quat2euler([self.uav.pos.rw,self.uav.pos.rx,self.uav.pos.ry,self.uav.pos.rz]) #wxyz default
            setpoint_yaw=euler.quat2euler([self.camera_setpoint.rw,self.camera_setpoint.rx,self.camera_setpoint.ry,self.camera_setpoint.rz]) #wxyz default

            # Switch to less aggressive controller when close
            if abs(self.camera_setpoint.x - self.uav.pos.x) < threshold_jog and abs(self.camera_setpoint.y-self.uav.pos.y) < threshold_jog and abs(self.camera_setpoint.z-self.uav.pos.z) < threshold_jog and degrees(abs(setpoint_yaw-current_yaw)) < threshold_jog_deg:
                rospy.loginfo_once("Setpoint[%s,%s,%s] close to drone, jogging it inwards based on past position",self.last_acceptable_setpoint.x,self.last_acceptable_setpoint.y,self.last_acceptable_setpoint.z)
                self.uav.setpoint_controller(self.camera_setpoint,"close")
                # TODO !!! self.thruster_controller.publish(self.last_acceptable_setpoint,self.local_position) # Change to output PWM
            
            # Approach setpoint with aggressive controller when far
            else:
                rospy.loginfo_once("Setpoint far from drone, using controller %s",self.local_position.x)
                self.last_acceptable_setpoint = self.camera_setpoint
                self.uav.setpoint_controller(self.camera_setpoint,"far")

            self.uav.setpoint(0,0,1) # Publish setpoint at x=0, y=0, z=1
            self.rosrate.sleep()

            

    def camera_listener_callback(self, msg):
        rospy.loginfo("New Camera setpoint(x:"+str(msg.pose.position.x)+", y:"+str(msg.pose.position.y)+", z:"+str(msg.pose.position.z)+")")
                # If TF is used as the as the position
        if msg._type=="tf2_msgs/TFMessage":
            if msg.transforms[0].header.frame_id == world_frame_id and msg.transforms[0].child_frame_id == camera_frame_id:
                self.camera_setpoint.x = msg.transforms[0].transform.translation.x
                self.camera_setpoint.y = msg.transforms[0].transform.translation.y
                self.camera_setpoint.z = msg.transforms[0].transform.translation.z
                self.camera_setpoint.rx = msg.transforms[0].transform.rotation.x
                self.camera_setpoint.ry = msg.transforms[0].transform.rotation.y
                self.camera_setpoint.rz = msg.transforms[0].transform.rotation.z
                self.camera_setpoint.rw = msg.transforms[0].transform.rotation.w
        elif msg._type=="geometry_msgs/PoseStamped":
            self.camera_setpoint.x = msg.pose.position.x
            self.camera_setpoint.y = msg.pose.position.y
            self.camera_setpoint.z = msg.pose.position.z
            self.camera_setpoint.rw = msg.pose.orientation.w
            self.camera_setpoint.rx = msg.pose.orientation.x
            self.camera_setpoint.ry = msg.pose.orientation.y
            self.camera_setpoint.rz = msg.pose.orientation.z
        else:
            rospy.logfatal("Invalid camera setpoint message type")

        #Perform transformation of camera setpoint wrt to body
        self.camera_setpoint.pose.position.x=self.camera_setpoint.pose.position.x-cameratobody_x
        self.camera_setpoint.pose.position.y=self.camera_setpoint.pose.position.y-cameratobody_y
        self.camera_setpoint.pose.position.z=self.camera_setpoint.pose.position.z-cameratobody_z
        
    def quit(self):
        print("Killing node")
        rospy.signal_shutdown("Node shutting down")


if __name__ == '__main__':
    
    rospy.init_node('Offboard_Wall_Node')

    node = offboard_node()

    rospy.spin()

