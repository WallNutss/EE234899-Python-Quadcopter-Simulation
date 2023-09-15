# -*- coding: utf-8 -*-

import numpy as np
from math import sin, cos, pi

# This is the rotation matrix containing rotation
# from the Body Frame --> Earth/Inertial Frame

def RPY2XYZ(angles): # ABrevation of Roll,Pitch,Yaw to XYZ Global Frame
    # Euler Angle in Body Frame / Other way around
    phi   = angles[0]
    theta = angles[1]
    psi   = angles[2]

    Rx = np.array([1,        0,         0, \
                   0, cos(phi), -sin(phi), \
                   0, sin(phi),  cos(phi) ]).reshape(3,3)
    
    Ry = np.array([cos(theta), 0, sin(theta), \
                            0, 1,         0,  \
                  -sin(theta), 0, cos(theta) ]).reshape(3,3)
    
    Rz = np.array([cos(psi), -sin(psi), 0, \
                   sin(psi),  cos(psi), 0, \
                          0,         0, 1  ]).reshape(3,3)
    
    # Rz*Ry*Rz = RX*RY*RZ // Body Frame to Inertial Frame Connection
    R = np.dot(Rz, np.dot(Ry,Rx))

    return R

def Deg2Rad(x):
    return (x/180)*pi

def Rad2Deg(x):
    return (x/pi)*180

