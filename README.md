# UAV_wrapper_py
Python wrapper for UAV's (PX4/Ardupilot) running ROS interfaces such as MAVROS



## Example Scripts

example.py - template for a basic hover in offboard

simple_survey.py - example usage of the continous_survey function for continous pattern flight



## Classes

uav - main class (from uav import uav)

uav_variables - class to hold variables as different data types have different structures (from uav import uav_variables)

controller - allows for controller to be used in addition to the built in mpc (from controller import controller), can be imported seperately from the uav class if needed



## Functions
self.[variable_name].setpoint(x,y,z) - send an x y z setpoint while mantaining current orientation

self.[variable_name].setpoint_yaw(x,y,z,yaw) - send cartesian and yaw setpoint

self.[variable_name].setpoint_quat(x,y,z,rx,ry,rw,rz) - send both pose and orientation setpoint

self.[variable_name].global(latitude,longitude,altitude) - send setpoint in latitude longitude and altitude setpoint



self.[variable_name].survey - conduct survey of array(initialised on creation of class with survey_array)

self.[variable_name].continous_survey - conduct survey of array(initialised on creation of class with survey_array_z)

self.[variable_name].continous_survey_update(self, array) - update continous survey array



self.[variable_name].init_controller(name, x_k, x_kd, y_kp, y_kd, z_kp, z_kd, yaw_kp, yaw_kd) - initalise controller, leave empty for unused fields

self.[variable_name].setpoint_controller(setpoint,controller_name) - send setpoints via controller



## Variables
self.[variable_name].pos.x - local position callback of uav in X axis

self.[variable_name].pos.rx - local position callback of uav in rx quat frame
