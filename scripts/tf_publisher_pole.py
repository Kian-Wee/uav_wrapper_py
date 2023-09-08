#!/usr/bin/env python

# Auxiliary script to add a tf transform between two points

import rospy
import tf2_ros
import math
import numpy as np
from geometry_msgs.msg import TransformStamped
from transforms3d import _gohlketransforms,euler
from math import degrees

class tf_publisher():

        def __init__(self, distance=0.5005, uav_frame_id="moose",camera_frame_id="pole",target_frame_id="body_setpoint", global_frame_id="map", hover_height=1.2):
            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tfBuffer)
            self.uav_to_body_setpoint_broadcaster = tf2_ros.TransformBroadcaster()
            self.uav_frame_id=uav_frame_id
            self.camera_frame_id=camera_frame_id
            self.target_frame_id=target_frame_id
            self.global_frame_id=global_frame_id
            self.distance=distance
            self.hover_height=hover_height

            rosrate=rospy.Rate(60)
            while not rospy.is_shutdown():
                self.transform()
                print("running")
                rosrate.sleep()


        def init_variables(self, distance=0.5005, uav_frame_id="moose",camera_frame_id="pole",target_frame_id="body_setpoint", global_frame_id="map", hover_height=1.2):
            self.uav_frame_id=uav_frame_id
            self.camera_frame_id=camera_frame_id
            self.target_frame_id=target_frame_id
            self.global_frame_id=global_frame_id
            self.distance=distance
            self.hover_height=hover_height

        def transform(self):
            # Constantly try to get a transform between the uav and the camera setpoint
            # If the setpoint exists, calculate new body setpoint 0.4m out of the wall
            try:


                newtransformStamped = TransformStamped()
                newtransformStamped.header.frame_id = self.camera_frame_id
                newtransformStamped.header.stamp = rospy.get_rostime()
                newtransformStamped.child_frame_id = self.target_frame_id


                # # Translation: Find vector from drone to new body setpoint by setting it cameratobody_x away from camera
                # transform_stamped2 = self.tfBuffer.lookup_transform(self.camera_frame_id, self.uav_frame_id, rospy.Time(0))
                # vector_array = [transform_stamped2.transform.translation.x, transform_stamped2.transform.translation.y, transform_stamped2.transform.translation.z]
                # mag = magnitude(np.array(vector_array))
                # trans_s_new = (self.distance) * (np.array(vector_array) /mag)
                # Send out transform again for visualisation
                newtransformStamped.transform.translation.x = 0
                newtransformStamped.transform.translation.y = 0
                newtransformStamped.transform.translation.z = 0.6


                # Rotation: No changes needed for rotation
                newtransformStamped.transform.rotation.x  = 0#transform_stamped.transform.rotation.x
                newtransformStamped.transform.rotation.y  = 0#transform_stamped.transform.rotation.y
                newtransformStamped.transform.rotation.z  = 0#transform_stamped.transform.rotation.z
                newtransformStamped.transform.rotation.w  = 1#transform_stamped.transform.rotation.w


                print("pub")


                self.uav_to_body_setpoint_broadcaster.sendTransform(newtransformStamped)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logdebug("Missing camera tf transform")

def magnitude(vector):
    return math.sqrt(sum(pow(element, 2) for element in vector))



if __name__ == '__main__':
    
    rospy.init_node('tf_publisher_node')

    node = tf_publisher()



    rospy.spin()
