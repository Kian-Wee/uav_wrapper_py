import rospy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from transforms3d import _gohlketransforms,euler
import tf
from controller import controller
import math
from pygeodesy.geoids import GeoidPGM
_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)


# This class is a wrapper to simplify common setpoint sending and position callbacks to make the main script more readable, espically when implementing other logic on the main script
# By default it works with local positioning in mavros but can be adapted for global positoning
class uav():

    # /mavros/local_position/pose for local indoor position; /mavros/global_position/local for local outdoor position with GPS
    def __init__(self,position_topic="mavros/local_position/pose",position_topic_type=PoseStamped,setpoint_topic="mavros/setpoint_position/local",setpoint_topic_type=PoseStamped,
                 name="",tf_world_frame="world",tf_drone_frame="drone",survey_array=[],global_survey_array=[],survey_array_cont=[],state_topic='mavros/state'):
        self.position_topic=name+position_topic
        self.setpoint_topic=name+setpoint_topic
        self.tf_world_frame=name+tf_world_frame
        self.tf_drone_frame=name+tf_drone_frame
        self.position_topic_type=position_topic_type
        self.setpoint_topic_type=setpoint_topic_type
        self.controller_array=[]
        self.survey_array=survey_array
        self.survey_array_z=survey_array_cont
        self.global_survey_array=global_survey_array
        self.continous_survey_pos=0
        self.state_topic=state_topic

        self.pos=uav_variables()
        rospy.Subscriber(
            self.position_topic,
            position_topic_type,
            self.position_listener_callback)
        
        self.global_pos=uav_variables()
        rospy.Subscriber("mavros/global_position/global",NavSatFix,self.global_pos_callback)
        
        self.setpoint_publisher = rospy.Publisher(self.setpoint_topic, setpoint_topic_type, queue_size=1)
        self.global_setpoint_publisher = rospy.Publisher("mavros/setpoint_position/global", GeoPoseStamped, queue_size=1)

        self.mode=''
        rospy.Subscriber(
            self.state_topic,
            State,
            self.state_listener_callback)


    def position_listener_callback(self,msg):
        if msg._type=="tf2_msgs/TFMessage":
            try:
                (trans,rot)=self.listener.lookupTransform(self.tf_drone_frame, self.tf_world_frame, rospy.Time(0))
                self.pos.x = trans[0]
                self.pos.y = trans[1]
                self.pos.z = trans[2]
                self.pos.rx = rot[0]
                self.pos.ry = rot[1]
                self.pos.rz = rot[2]
                self.pos.rw = rot[3]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logdebug("Missing tf transform")
        elif msg._type=="geometry_msgs/PoseStamped":
            self.pos.x = msg.pose.position.x
            self.pos.y = msg.pose.position.y
            self.pos.z = msg.pose.position.z
            self.pos.rw = msg.pose.orientation.w
            self.pos.rx = msg.pose.orientation.x
            self.pos.ry = msg.pose.orientation.y
            self.pos.rz = msg.pose.orientation.z
        elif msg._type== "nav_msgs/Odometry" or msg._type== "geometry_msgs/PoseWithCovarianceStamped":
            self.pos.x=msg.pose.pose.position.x
            self.pos.y=msg.pose.pose.position.y
            self.pos.z=msg.pose.pose.position.z
            self.pos.rx=msg.pose.pose.orientation.x
            self.pos.ry=msg.pose.pose.orientation.y
            self.pos.rz=msg.pose.pose.orientation.z
            self.pos.rw=msg.pose.pose.orientation.w
        elif msg._type=="sensor_msgs/NatSatFix":
            rospy.loginfo_once("Using global positioning for position source")
            self.pos.x=msg.latitude
            self.pos.y=msg.longitude
            self.pos.z=msg.altitude
            # self.pos.rx=msg.pose.pose.orientation.x
            # self.pos.ry=msg.pose.pose.orientation.y
            # self.pos.rz=msg.pose.pose.orientation.z
            # self.pos.rw=msg.pose.pose.orientation.w
        else:
            rospy.logfatal("Invalid/Unsupported local position message type")


    def state_listener_callback(self,msg):
        self.mode = str(msg.mode)

    # Send setpoint directly to px4's MPC controller without any rotation
    # Roll and pitch are assumed to be 0 for simplicity as /setpoint_position/local does not take it into account anyways
    def setpoint(self,x,y,z):
        if self.setpoint_topic_type == PoseStamped:
            msg = PoseStamped()
            msg.header.frame_id="map"
            # msg.header.stamp = rospy.get_time()
            msg.pose.position.x= x
            msg.pose.position.y= y
            msg.pose.position.z= z
            msg.pose.orientation.w = self.pos.rw
            msg.pose.orientation.x = self.pos.rx
            msg.pose.orientation.y = self.pos.ry
            msg.pose.orientation.z = self.pos.rz
            self.setpoint_publisher.publish(msg)
        elif self.setpoint_topic_type == GeoPoseStamped:
            if self.position_topic_type != NavSatFix:
                rospy.logfatal("Using Global Setpoints but the positioning topic is not Global!!!")
            else:
                rospy.loginfo_once("Using global setpoints")
                msg = GeoPoseStamped()
                msg.pose.position.latitude=x
                msg.pose.position.longitude=y
                msg.pose.position.altitude=z
                # No yaw avaliable for the global_position/global topic
                # msg.pose.orientation.w = self.pos.rw
                # msg.pose.orientation.x = self.pos.rx
                # msg.pose.orientation.y = self.pos.ry
                # msg.pose.orientation.z = self.pos.rz
        else:
            rospy.logfatal("Invalid/Unsupported setpoint position message type")


    def setpoint_global(self,x,y,z):
        msg = GeoPoseStamped()
        msg.header.frame_id="map"
        msg.pose.position.latitude=x
        msg.pose.position.longitude=y
        msg.pose.position.altitude=z
        msg.pose.orientation.w = self.pos.rw
        msg.pose.orientation.x = self.pos.rx
        msg.pose.orientation.y = self.pos.ry
        msg.pose.orientation.z = self.pos.rz
        self.global_setpoint_publisher.publish(msg)


    def global_pos_callback(self,msg):
        self.global_pos.x = msg.latitude
        self.global_pos.y = msg.longitude
        self.global_pos.z = msg.altitude -_egm96.height(msg.latitude, msg.longitude)

    
    # Global setpoint survey through an array
    # More info on threshold here: http://wiki.gis.com/wiki/index.php/Decimal_degrees
    def global_survey(self, threshold = 0.0000001):

        # XY coordinates given, z assmed to be at current height
        if len(self.global_survey_array[0]) == 2:
            if len(self.global_survey_array) != 0:
                self.setpoint_global(self.global_survey_array[0][0],self.global_survey_array[0][1],self.global_pos.z) # Return first position in the array
                if abs(self.global_survey_array[0][0] - self.global_pos.x) < threshold and abs(self.global_survey_array[0][1] - self.global_pos.y) < threshold:
                    rospy.loginfo("Currently at waypoint %s, [%s]",str(len(self.global_survey_array)),str(self.global_survey_array[0]))
                    self.global_survey_array.pop(0)
                return 1
            else:
                self.setpoint_global(self.global_pos.x,self.global_pos.y,self.global_pos.z)
                return 0 # Ended
            
        # XYZ coordinates given
        elif len(self.global_survey_array[0]) == 1:
                if len(self.global_survey_array) != 0:
                    self.setpoint_global(self.global_survey_array[0][0],self.global_survey_array[0][1],self.global_survey_array[0][2]) # Return first position in the array
                    if abs(self.global_survey_array[0][0] - self.global_pos.x) < threshold and abs(self.global_survey_array[0][1] - self.global_pos.y) < threshold and abs(self.global_survey_array[0][2] - self.global_pos.z) < threshold:
                        rospy.loginfo("Currently at waypoint %s, [%s]",str(len(self.global_survey_array)),str(self.global_survey_array[0]))
                        self.global_survey_array.pop(0)
                    return 1
                else:
                    self.setpoint_global(self.global_pos.x,self.global_pos.y,self.global_pos.z)
                    return 0 # Ended
                
        else:
            rospy.logfatal_throttle_identical(1,"Invalid global survey array type")
        
    # Local setpoint survey through an array
    def survey(self, threshold = 0.1):

        if len(self.survey_array[0]) == 2:
            if len(self.survey_array) != 0:
                self.setpoint(self.survey_array[0][0],self.survey_array[0][1],self.pos.z) # Return first position in the array
                if abs(self.survey_array[0][0] - self.pos.x) < threshold and abs(self.survey_array[0][1] - self.pos.y) < threshold:
                    rospy.loginfo("Currently at waypoint %s, [%s]",str(len(self.survey_array)),str(self.survey_array[0]))
                    self.survey_array.pop(0)
                return 1
            else:
                self.setpoint(self.pos.x,self.pos.y,self.pos.z)
                return 0 # Ended
            
        elif len(self.survey_array[0]) == 3:
            if len(self.survey_array) != 0:
                self.setpoint(self.survey_array[0][0],self.survey_array[0][1],self.survey_array[0][2]) # Return first position in the array
                if abs(self.survey_array[0][0] - self.pos.x) < threshold and abs(self.survey_array[0][1] - self.pos.y) < threshold and abs(self.survey_array[0][2] - self.pos.z) < threshold:
                    rospy.loginfo("Currently at waypoint %s, [%s]",str(len(self.survey_array)),str(self.survey_array[0]))
                    self.survey_array.pop(0)
                return 1
            else:
                self.setpoint(self.pos.x,self.pos.y,self.pos.z)
                return 0 # Ended
            
        else:
            rospy.logfatal_throttle_identical(1,"Invalid local survey array type")

    # Contionus Local setpoint survey through an array
    def continous_survey(self, threshold = 0.1):

        # rospy.logerr("Currently at waypoint %s",str(self.continous_survey_pos)) #####
        if len(self.survey_array_z[0]) == 2:
            self.setpoint(self.survey_array_z[self.continous_survey_pos][0],self.survey_array_z[self.continous_survey_pos][1],self.pos.z) # Return first position in the array
            if abs(float(self.survey_array_z[self.continous_survey_pos][0] - self.pos.x)) < threshold and abs(float(self.survey_array_z[self.continous_survey_pos][1]) - self.pos.y) < threshold:
                if self.continous_survey_pos == (len(self.survey_array_z)-1):
                    rospy.loginfo("Currently at waypoint %s, resetting to 0",str(self.continous_survey_pos))
                    self.continous_survey_pos = 0
                else:
                    rospy.loginfo("Currently at waypoint %s, moving to next waypoint",str(self.continous_survey_pos))
                    self.continous_survey_pos = self.continous_survey_pos + 1 # += doesnt seem to work properly

        elif len(self.survey_array_z[0]) == 3:
            self.setpoint(self.survey_array_z[self.continous_survey_pos][0],self.survey_array_z[self.continous_survey_pos][1],self.survey_array_z[self.continous_survey_pos][2]) # Return first position in the array
            if abs(float(self.survey_array_z[self.continous_survey_pos][0] - self.pos.x)) < threshold and abs(float(self.survey_array_z[self.continous_survey_pos][1]) - self.pos.y) < threshold and abs(float(self.survey_array_z[self.continous_survey_pos][2] - self.pos.z)) < threshold:
                if self.continous_survey_pos == (len(self.survey_array_z)-1):
                    rospy.loginfo("Currently at waypoint %s, resetting to 0",str(self.continous_survey_pos))
                    self.continous_survey_pos = 0
                else:
                    rospy.loginfo("Currently at waypoint %s, moving to next waypoint",str(self.continous_survey_pos))
                    self.continous_survey_pos = self.continous_survey_pos + 1 # += doesnt seem to work properly

        else:
            rospy.logfatal_throttle_identical(1,"Invalid continous survey array type")


    def continous_survey_update(self, array):
        self.survey_array_z = array


    # Send setpoint directly to px4's MPC controller in euler:yaw(in degrees)
    def setpoint_yaw(self,x,y,z,yaw):
        if self.setpoint_topic_type == PoseStamped:
            msg = PoseStamped()
            msg.header.frame_id="map"
            # msg.header.stamp = rospy.get_time()
            msg.pose.position.x= x
            msg.pose.position.y= y
            msg.pose.position.z= z
            q = _gohlketransforms.quaternion_from_euler(0, 0, math.radians(yaw), 'ryxz')
            msg.pose.orientation.w = q[0]
            msg.pose.orientation.x = q[1]
            msg.pose.orientation.y = q[2]
            msg.pose.orientation.z = q[3]
            self.setpoint_publisher.publish(msg)
        else:
            rospy.logfatal("Invalid/Unsupported setpoint position message type")


    # Send setpoint directly to px4's MPC controller in quarternion
    def setpoint_quat(self,x,y,z,rx,ry,rz,rw):
        if self.setpoint_topic_type == PoseStamped:
            msg = PoseStamped()
            msg.header.frame_id="map"
            # msg.header.stamp = rospy.get_time()
            msg.pose.position.x= x
            msg.pose.position.y= y
            msg.pose.position.z= z
            msg.pose.orientation.w = rw
            msg.pose.orientation.x = rx
            msg.pose.orientation.y = ry
            msg.pose.orientation.z = rz
            self.setpoint_publisher.publish(msg)
        else:
            rospy.logfatal("Invalid/Unsupported setpoint position message type")


    # Initalise additional MPC controller (pre-PX4 MPC)
    def init_controller(self, name, x_kp=0, x_kd=0, y_kp=0, y_kd=0, z_kp=0, z_kd=0, yaw_kp=0, yaw_kd=0):
        # self.controller_array.append(controller.controller.__init_subclass__(name, x_kp, x_kd, y_kp, y_kd, z_kp, z_kd, yaw_kp, yaw_kd))
        self.controller_array.append(controller(name, x_kp, x_kd, y_kp, y_kd, z_kp, z_kd, yaw_kp, yaw_kd))


    # Send setpoint, running through controller
    def setpoint_controller(self,setpoint,controller_name):
        check = False
        for i in self.controller_array:
            if i.name == controller_name:
                arr=i.controller(setpoint,self.pos)
                self.setpoint_quat(arr[0],arr[1],arr[2],arr[3],arr[4],arr[5],arr[6])
                check = True
        if check == False:
            rospy.logfatal("Missing setpoint controller")


# UAV class to hold variables
# The main purpose of this class abstract out only the relevant position and orientation fields irregardless of whatever message type is used
# The different message types are stored as these class variables to make it easier and more consistent to read and access
class uav_variables():

    def __init__(self,x=0,y=0,z=0,rx=0,ry=0,rz=0,rw=1):
        self.x=x
        self.y=y
        self.z=z
        self.rx=rx
        self.ry=ry
        self.rz=rz
        self.rw=rw

    # Save a tf2 lookup into local variable
    def save_tf2(self,transform_stamped):
        self.x=transform_stamped.transform.translation.x
        self.y=transform_stamped.transform.translation.y
        self.z=transform_stamped.transform.translation.z
        self.rx=transform_stamped.transform.rotation.x
        self.ry=transform_stamped.transform.rotation.y
        self.rz=transform_stamped.transform.rotation.z
        self.rw=transform_stamped.transform.rotation.w

    def update(self,x,y,z,rx,ry,rz,rw):
        self.x=x
        self.y=y
        self.z=z
        self.rx=rx
        self.ry=ry
        self.rz=rz
        self.rw=rw

