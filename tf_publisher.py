#!/usr/bin/env python

# Auxiliary script to add a tf transform between two points

import rospy
import tf2_ros
import math
import numpy as np
from geometry_msgs.msg import TransformStamped

class tf_publisher():

        def __init__(self, distance, uav_frame_id="moose",camera_frame_id="camera",target_frame_id="body_setpoint", global_frame_id="map", hover_height=1.2):
            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tfBuffer)
            self.uav_to_body_setpoint_broadcaster = tf2_ros.TransformBroadcaster()
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

                # # No changes needed for rotation
                # transform_stamped = self.tfBuffer.lookup_transform("camera", "map", rospy.Time(0))
                # self.camera_setpoint.rx = transform_stamped.transform.rotation.x
                # self.camera_setpoint.ry = transform_stamped.transform.rotation.y
                # self.camera_setpoint.rz = transform_stamped.transform.rotation.z
                # self.camera_setpoint.rw = transform_stamped.transform.rotation.w

                # Find vector from drone to new body setpoint by setting it cameratobody_x away from camera
                # (trans_s,rot_s)=self.listener.lookupTransform(camera_frame_id, uav_frame_id, rospy.Time(0)) #to frame, from frame, time
                transform_stamped2 = self.tfBuffer.lookup_transform(self.camera_frame_id, self.uav_frame_id, rospy.Time(0))


                # mag = magnitude(np.array(trans_s))
                # trans_s_new = ( 1 - cameratobody_x/mag) * np.array(trans_s) / mag
                vector_array = [transform_stamped2.transform.translation.x, transform_stamped2.transform.translation.y, transform_stamped2.transform.translation.z]
                mag = magnitude(np.array(vector_array))
                trans_s_new = ( 1 - self.distance/mag) * np.array(vector_array) / mag

                # Send out transform again for visualisation
                # self.uav_to_body_setpoint_broadcaster.sendTransform(trans_s_new, rot_s, rospy.Time.now(), "body_setpoint", "camera")
                newtransformStamped = TransformStamped()
                newtransformStamped.header.frame_id = self.camera_frame_id
                newtransformStamped.header.stamp = rospy.get_rostime()
                newtransformStamped.child_frame_id = self.target_frame_id
                newtransformStamped.transform.translation.x = trans_s_new[0]
                newtransformStamped.transform.translation.y = trans_s_new[1]
                newtransformStamped.transform.translation.z = trans_s_new[2]
                newtransformStamped.transform.rotation = transform_stamped2.transform.rotation
                self.uav_to_body_setpoint_broadcaster.sendTransform(newtransformStamped)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logdebug("Missing camera tf transform")

def magnitude(vector):
    return math.sqrt(sum(pow(element, 2) for element in vector))