#!/usr/bin/env python

import numpy as np
from transforms3d import _gohlketransforms,euler


# This subclass may be instantiated multiple times by the uav class to add multiple controllers to the offboard script
class controller():

    def __init_subclass__(self,name, x_kp, x_kd, y_kp, y_kd, z_kp, z_kd, yaw_kp, yaw_kd, **kwargs):
        super().__init_subclass__(**kwargs)
        print(f"Called __init_subclass({self}, {name})")
        self.name = name
        self.x= self.pid_variables(x_kp,0,x_kd)
        self.y= self.pid_variables(y_kp,0,y_kd)
        self.z= self.pid_variables(z_kp,0,z_kd)
        self.yaw= self.pid_variables(yaw_kp,0,yaw_kd)
        self.error_past=0
        

    def controller(self, setpoint, current):
        # Error = Setpoint - Feedback
        self.error = np.subtract(np.array([setpoint.x, setpoint.y, setpoint.z,
                                        euler.quat2euler([setpoint.rw,setpoint.rx,setpoint.ry,setpoint.rz])[2]]),
                                        np.array([current.x, current.y, current.z,
                                        euler.quat2euler([current.rw,current.rx,current.ry,current.rz])[2]]) )
        # Derivative error = Error - error_past
        self.derivative_error = self.error - self.error_past

        input_x = (self.x.p * self.error[0]) + (self.x.d * self.derivative_error[0])
        input_y = (self.y.p * self.error[1]) + (self.y.d * self.derivative_error[1])
        input_z = ((self.z.p * self.error[2]) + (self.z.d * self.derivative_error[2]))
        input_eul = (self.yaw.p * self.error[3]) + (self.yaw.d * self.derivative_error[3])
        q = _gohlketransforms.quaternion_from_euler(0, 0, input_eul, 'ryxz') # 0 pitch, 0 roll

        self.error_past = self.error
        
        return[input_x+current.x,input_y+current.y,input_z+current.z,
                            q[0],q[1],q[2],q[3]]


    class pid_variables():

        def __init__(self, p, i, d):
            self.p=p
            self.i=i
            self.d=d