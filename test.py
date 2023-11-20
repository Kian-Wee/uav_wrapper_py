#!/usr/bin/env python3
import math
import numpy as np

rate = 60
beginsweep = 0
sweeparr=[]
phase = ""

def slowyaw(angle=720, w=10):
    global rate, beginsweep, sweeparr, phase
    print("rate",rate)

    yaw = 0

    if beginsweep==0 and int(yaw - angle) != 0:
        sweeparr=[]
        print("Yawing from {0} to {1}".format(yaw,angle))

        totaldeg=angle-yaw
        numofsteps=round(totaldeg/w*rate)
        print(totaldeg,numofsteps)
        print(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps)
        if numofsteps == 0: # This happens if the angle is small and by the end of one rate it will hit the target, so just send the final angle
            sweeparr += [angle] * rate # Add 1s to turn to final direction
        else:
            if totaldeg > 0:
                for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps):
                    sweeparr.append(i)
                    # print(i)
            else:
                for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): sweeparr.append(i)
                    
        sweeparr += [sweeparr[-1]] * rate # Add 1s to turn to final direction
        beginsweep=1

        print(sweeparr)
        print(len(sweeparr))
        print(len(sweeparr)/rate)
    
    # Should never be invoked, left for debugging
    elif beginsweep==0 and (yaw - angle) == 0:
        print("Not sweeping as provided angle is the same as current heading")
        if phase=="Sweep":  phase="Idle"
        return angle
    elif beginsweep==0:
        print("This message should not be printing. It means that the sweep array is not created properly and it is Not Sweeping.")
        if phase=="Sweep":  phase="Idle"
        return angle
    else:
        if sweeparr==[]:
            if phase == "Sweep": # In go there, swap the mode change
                print("Sweep ended")
                phase="Idle"
            return angle
            # return math.degrees(euler_from_quaternion([self.uav_pos.rx,self.uav_pos.ry,self.uav_pos.rz,self.uav_pos.rw])[2]) # return the current position (else it defaults to 0)
    
    desiredyaw=sweeparr[0]
    sweeparr.pop(0)
    return desiredyaw


slowyaw(angle=720,w=10)