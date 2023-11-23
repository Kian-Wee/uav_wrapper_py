#!/usr/bin/env python3

import rospy
import numpy as np
from uav import uav,uav_variables
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest, CommandLong
import math
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

resume_odom_srv = rospy.ServiceProxy('/nightray/resume_odom', Empty) # Resume odometry
resume_srv = rospy.ServiceProxy('/nightray/resume', Empty) # Resume mapping

#take off and yaw test
# Just wait for 2 signals, arm, takeoff, spin 3 times and land

# Update rate
rate = 60 #60 times every second

class offboard_node():

    def __init__(self): 
        print("Initalising Controller")

        self.uav = uav(position_topic="nightray/mavros/local_position/pose",setpoint_topic="nightray/mavros/setpoint_position/local",state_topic='nightray/mavros/state')

        self.rosrate=rospy.Rate(rate)
        rospy.on_shutdown(self.quit)

        self.phase = "uninit"

        rospy.Subscriber(
        "/mapping_takeoff",
        Bool,
        self.start_callback)
        self.init = 0

        rospy.wait_for_service("nightray/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("nightray/mavros/cmd/arming", CommandBool)
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        self.takeoff_pos=[0,0,1]
        self.hover_pos=[-0.5,0,1]
        self.threshold = 0.1 #m

        self.sweep_pos=uav_variables()
        self.beginsweep=0
        self.sweeparr=[]

        flight_mode_srv = rospy.ServiceProxy('nightray/mavros/set_mode', SetMode)

        rospy.Subscriber(
        "/nightray/prearm_check_ready",
        Bool,
        self.prearm_check_callback)
        self.prearm_check=0
        
        self.prearm_reboot_srv = rospy.ServiceProxy('/nightray/mavros/cmd/command', CommandLong) #"broadcast: false, command: 246, confirmation: 0, param1: 1.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0"

        while not rospy.is_shutdown():

            # Check for init signal
            if self.init == 1:

                # Wait for drone flags to clear(fusion of pose)
                if self.prearm_check == 1:
                    # print("Come to me geo daddy")
                    # Set to offboard mode
                    if self.uav.mode=='OFFBOARD':
                        
                        # Arm the drone
                        if self.phase == "waiting":

                            if(arming_client.call(arm_cmd).success == True):
                                rospy.loginfo("Vehicle armed")
                                self.phase="armed"
                                #Set subsequent setpoints with respect to current position
                                self.takeoff_pos[0] = self.takeoff_pos[0] + self.uav.pos.x
                                self.takeoff_pos[1] = self.takeoff_pos[1] + self.uav.pos.y
                                self.takeoff_pos[2] = self.takeoff_pos[2] + self.uav.pos.z
                                self.hover_pos[0] = self.hover_pos[0] + self.uav.pos.x
                                self.hover_pos[1] = self.hover_pos[1] + self.uav.pos.y
                                self.hover_pos[2] = self.hover_pos[2] + self.uav.pos.z

                        elif self.phase == "armed":
                            rospy.loginfo_throttle(2,"Taking off to setpoint %s",str(self.takeoff_pos))
                            self.uav.setpoint(self.takeoff_pos[0],self.takeoff_pos[1],self.takeoff_pos[2]) # Publish setpoint at x=0, y=0, z=1
                            if abs(self.uav.pos.x - self.takeoff_pos[0]) < self.threshold and abs(self.uav.pos.y - self.takeoff_pos[1]) < self.threshold and abs(self.uav.pos.z - self.takeoff_pos[2]) < self.threshold:
                                rospy.loginfo_once("At take-off setpoint %s, Moving forward",str(self.takeoff_pos))
                                self.phase="Moving"

                        elif self.phase == "Moving":
                            rospy.loginfo_throttle(2,"Moving to setpoint %s",str(self.hover_pos))
                            self.uav.setpoint(self.hover_pos[0],self.hover_pos[1],self.hover_pos[2])
                            if abs(self.uav.pos.x - self.hover_pos[0]) < self.threshold and abs(self.uav.pos.y - self.hover_pos[1]) < self.threshold and abs(self.uav.pos.z - self.hover_pos[2]) < self.threshold:
                                rospy.loginfo_once("At hover setpoint %s, Sweeping",str(self.takeoff_pos))
                                self.phase="Sweep"
                                self.sweep_pos.x=self.uav.pos.x
                                self.sweep_pos.y=self.uav.pos.y
                                self.sweep_pos.z=self.uav.pos.z

                        elif self.phase == "Sweep":
                            rospy.loginfo_throttle(2,"Sweeping")
                            self.uav.setpoint_yaw(self.sweep_pos.x,self.sweep_pos.y,self.sweep_pos.z,self.slowyaw())

                        elif self.phase == "Idle":
                            if(flight_mode_srv(custom_mode='AUTO.LAND')):
                                rospy.loginfo_throttle(2,"land success")
                        else:
                            rospy.loginfo_throttle(2,"No command, hovering at current position")
                            self.uav.setpoint(self.uav.pos.x,self.uav.pos.y,self.uav.pos.z)
                    else:
                        self.uav.setpoint(self.uav.pos.x,self.uav.pos.y,self.uav.pos.z) # Pub position callback to allow it to boot into offboard


            self.rosrate.sleep()
        
    def quit(self):
        print("Killing node")
        rospy.signal_shutdown("Node shutting down")

    def prearm_check_callback(self,msg):
        if msg.data == 1 and self.prearm_check == 0 and self.init == 1:
            rospy.loginfo("Pre arm check sucessful, waiting for offboard mode to proceed to arming/takeoff")
            self.prearm_check = 1
            self.phase = "waiting"

    def start_callback(self, msg):
        if msg.data == 1 and self.init == 0:
            rospy.loginfo("Mapping takeoff start signal recieved")
            self.init = 1
            if(not resume_odom_srv()):
                rospy.logerr("Failed to resume odom!")
            if(not resume_srv()):
                rospy.logerr("Failed to resume map!")
            self.prearm_reboot_srv(command=246,param1=1)

    # Slows down sweep to a slower predefined speed, function is made to be non-blocking and returns a slowed down yaw without altering the position
    # w is angular velocity in degrees per second
    # Not inputting an angle(or putting in 720 degrees) defaults it to a auto sweep mode
    # Take note that both the inputs and the outputs are in DEGREES, not radians
    def slowyaw(self, angle=720, w=20):
        global rate

        yaw = math.degrees(euler_from_quaternion([self.uav.pos.rx, self.uav.pos.ry, self.uav.pos.rz, self.uav.pos.rw])[2])

        if self.beginsweep==0 and int(yaw - angle) != 0:
            self.sweeparr=[]
            print("Yawing from {0} to {1}".format(yaw,angle))

            totaldeg=angle-yaw
            numofsteps=round(totaldeg/w*rate)
            if numofsteps == 0: # This happens if the angle is small and by the end of one rate it will hit the target, so just send the final angle
                self.sweeparr += [angle] * rate # Add 1s to turn to final direction
            else:
                if totaldeg > 0:
                    for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps): self.sweeparr.append(i)
                else:
                    for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): self.sweeparr.append(i)
                        
            self.sweeparr += [self.sweeparr[-1]] * rate # Add 1s to turn to final direction
            self.beginsweep=1

        else:
            if self.sweeparr==[]:
                print("Sweep ended", yaw)
                self.phase="Idle"
                return yaw

        desiredyaw=self.sweeparr[0]
        self.sweeparr.pop(0)
        return desiredyaw


if __name__ == '__main__':
    
    rospy.init_node('Offboard_Node')

    node = offboard_node()

    rospy.spin()
