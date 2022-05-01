#!/usr/bin/env python3
import numpy as np

def quaternion(roll_inp,pitch_inp,yaw_inp):
    # all inputs are in radian

    cr = np.cos(roll_inp*0.5)
    sr = np.sin(roll_inp*0.5)
    cp = np.cos(pitch_inp*0.5)
    sp = np.sin(pitch_inp*0.5)
    cy = np.cos(yaw_inp*0.5)
    sy = np.sin(yaw_inp*0.5)

    W = cy*cr*cp + sy*sr*sp
    X = cy*sr*cp - sy*cr*sp
    Y = cy*cr*sp + sy*sr*cp
    Z = sy*cr*cp - cy*sr*sp

    q = np.array([W,X,Y,Z])

    return q
