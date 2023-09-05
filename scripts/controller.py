import numpy as np
from transforms3d import _gohlketransforms,euler
from scipy.spatial.transform import Rotation

# This subclass may be instantiated multiple times by the uav class to add multiple controllers to the offboard script
class controller():

    # def __init_subclass__(self,name, x_kp, x_kd, y_kp, y_kd, z_kp, z_kd, yaw_kp, yaw_kd, **kwargs):
    #     super().__init_subclass__(**kwargs)
    #     print(f"Called __init_subclass({self}, {name})")
    #     self.name = name
    #     self.x= self.pid_variables(x_kp,0,x_kd)
    #     self.y= self.pid_variables(y_kp,0,y_kd)
    #     self.z= self.pid_variables(z_kp,0,z_kd)
    #     self.yaw= self.pid_variables(yaw_kp,0,yaw_kd)
    #     self.error_past=0


    def __init__(self,name, x_kp, x_kd, y_kp, y_kd, z_kp, z_kd, yaw_kp, yaw_kd):
        self.name = name
        self.x= self.pid_variables(x_kp,0,x_kd)
        self.y= self.pid_variables(y_kp,0,y_kd)
        self.z= self.pid_variables(z_kp,0,z_kd)
        self.yaw= self.pid_variables(yaw_kp,0,yaw_kd)
        self.error_past=[0.0, 0.0, 0.0, 0.0]

    def controller(self, setpoint, current):
        # Error = Setpoint - Feedback
        Rset=Rotation.from_quat([setpoint.rx,setpoint.ry,setpoint.rz,setpoint.rw])
        Rcur=Rotation.from_quat([current.rx,current.ry,current.rz,current.rw])
        error = np.subtract(np.array([setpoint.x, setpoint.y, setpoint.z,
                                        Rset.as_euler("ZYX",degrees=True)[0]]),
                                        np.array([current.x, current.y, current.z,
                                        Rcur.as_euler("ZYX",degrees=True)[0]]) )
        
        R_ab = Rcur.inv()*Rset
        R_err = np.linalg.norm(R_ab.as_rotvec(degrees=True))

        # r_ab = np.matmul(np.transpose(Rcur),Rset) 
        # rot_error = np.rad2deg(np.arccos((np.trace(r_ab)-1)/2))
        # print("Matt error: %s",R_err)
        # print("Kian error: %s",rot_error)


        # Derivative error = Error - error_past
        # print(error, self.error_past)
        # derivative_error = float(np.array(np.zeros(4)))
        derivative_error = np.subtract(error, self.error_past)
        # print(derivative_error)


        input_x = (self.x.p * error[0]) + (self.x.d * derivative_error[0])
        input_y = (self.y.p * error[1]) + (self.y.d * derivative_error[1])
        input_z = ((self.z.p * error[2]) + (self.z.d * derivative_error[2]))
        # input_eul = (self.yaw.p * self.error[3]) + (self.yaw.d * self.derivative_error[3])

        # q = _gohlketransforms.quaternion_from_euler(0, 0, input_eul, 'ryxz') # 0 pitch, 0 roll 'ryxz'
        # print(derivative_error,derivative_error[3])
        derivative_error[3] = R_err - self.error_past[3]
        input_eul = (self.yaw.p * R_err) + (self.yaw.d * derivative_error[3])
        # print(input_eul)
        q= Rotation.from_euler("z",input_eul, degrees=True)
        # print(q)
        # print(current.rx,current.ry,current.rz,current.rw)
        # print(q[0]*current.rx,q[1]*current.ry,q[2]*current.rz,q[3]*current.rw)
        current_q = Rotation.from_quat([current.rx,current.ry,current.rz,current.rw])
        new_q = q*current_q

        self.error_past = error
        return[input_x+current.x,input_y+current.y,input_z+current.z,
               setpoint.rx,setpoint.ry,setpoint.rz,setpoint.rw] 
    #new_q.as_quat()[0],new_q.as_quat()[1],new_q.as_quat()[2],new_q.as_quat()[3]
                    # q[0]*current.rx,q[1]*current.ry,q[2]*current.rz,q[3]*current.rw        ] #rx,ry,rz,rw setpoint.rx,setpoint.ry,setpoint.rz,setpoint.rw
    

    # controller with 1 actuator output to control auxiliary devices which require pd control
    # Takes in single error input & single current state and returns single error output
    # Uses only x_kp and x_kd 
    def custom_single_controller(self, error, current):
        # Error = Setpoint - Feedback
        self.error = error
        # Derivative error = Error - error_past
        self.derivative_error = self.error - self.error_past

        input_a = 0.4 - (self.x.p * -self.error) + (self.x.d * self.derivative_error)

        self.error_past = self.error

        return[input_a+current]

    # # controller with 1 actuator output to control auxiliary devices which require pd control
    # # Takes in single error input & single current state and returns single error output
    # # Uses only x_kp and x_kd 
    # def custom_single_controller(self, error, current):
    #     # Error = Setpoint - Feedback
    #     self.error = error
    #     # Derivative error = Error - error_past
    #     self.derivative_error = self.error - self.error_past

    #     input_a = (self.x.p * self.error) + (self.x.d * self.derivative_error)

    #     self.error_past = self.error

    #     return[input_a+current]

    class pid_variables():

        def __init__(self, p, i, d):
            self.p=p
            self.i=i
            self.d=d
