# -*- coding: utf-8 -*-

import numpy as np
from math import sin, cos, tan, pi

# This is the rotation matrix containing rotation
# from the Body Frame --> Earth/Inertial Frame

def RPY2XYZ(angles): # Abrevation of Roll,Pitch,Yaw from Body Frame to XYZ Global Frame
    """
    Perform a rotation matrix calculation based on the body Frame Rz, Ry, Rx with order of calculation respective of that
    Args
        Angles[0] = Roll: Angular position about the x-axis in radians.
        Angles[1] = Pitch: Angular position about the y-axis in radians.
        Angles[2] =Yaw: Angular position about the z-axis in radians.
    Return
        3x3 rotation matrix as NumPy array
    """
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

def D2R(x):
    """
    Degree to Radian Function
    Args
        Degree
    Return
        Radian
    """
    return (x/180)*pi

def R2D(x):
    """
    Degree to Radian Function
    Args
        Radian
    Return
        Degree
    """
    return (x/pi)*180

def SpecialR(angles):
    """
    Perform a Special Rotation Matrix for Calculating Angular Rates in Body Frame to the Inertial Frame
    Args
        Angles[0] = Roll: Angular position about the x-axis in radians.
        Angles[1] = Pitch: Angular position about the y-axis in radians.
        Angles[2] =Yaw: Angular position about the z-axis in radians.
    Return
        3x3 special rotation matrix as NumPy array
    """
    phi   = angles[0]
    theta = angles[1]
    psi   = angles[2]
    Is = np.array([1, sin(phi)*tan(theta), cos(phi)*tan(theta), \
                   0,            cos(phi),           -sin(phi), \
                   0, sin(phi)/cos(theta), cos(phi)/cos(theta)]).reshape(3,3)
    return Is