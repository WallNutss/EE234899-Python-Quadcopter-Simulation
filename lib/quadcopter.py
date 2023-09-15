import numpy as np
import math

class quadcopter:
    def __init__(self, Ts = 1.0/50.0):

        # Tello Params
        self.g      = 9.81              # m/s^2             , gravity acceleration
        self.m      = 0.08              # kg                , mass of the drone
        self.l      = 0.06              # m                 , length of the each arm
        self.Ixx    = 0.0068            # kgm^2             , Symetrical Inertia matrix at x-axis
        self.Iyy    = 0.0068            # kgm^2             , Symetrical Inertia matrix at y-axis
        self.Izz    = 0.01313           # kgm^2             , Symetrical Inertia matrix at z-axis
        self.Ir     = 0.0000495         # kgm^2             , The moment of inertia of each rotor
        self.b      = 0.00000566        # kgm/ (rad)^2      , Thrust Constant
        self.d      = 0.0000000580644   # kgm^2/ (rad)^2    , Drag factor

        # Motor Params (Speed, rpm, etc)
        self.max_motor_speed = 2200.0   # rpm , max motor speed rotation per minute
        self.min_motor_speed = 0.0      # rpm , min motor speed rotation per minute
        self.u1_max          = 109.57   # N
        self.u1_min          = 0.0      # N
        self.u2_max          = 2.32     # Nm
        self.u2_min          = -2.32    # Nm
        self.u3_max          = 2.32     # Nm
        self.u3_min          = -2.32    # Nm
        self.u4_max          = 0.73     # Nm
        self.u4_min          = -0.73    # Nm

        # States of the Drone (Position, Orientation, Attitude)
        self.Ts     = Ts                # s     , Time sampling of the simulation
        self.state  = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(12,1)       # X,Y,Z, dX,dY,dZ, varphi,theta,psi, p,q,r
        '''
        state is the thing we measured. X,Y,Z means it on global frame where x,y,z is on the body frame
        '''
        self.dstate = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(12,1)       # The derivative of the state
        self.u      = np.array([self.u1_min, self.u2_min, self.u3_min, self.u4_min]).reshape(4.1) # Control Inputs for the dynamics mathematical model after
        self.T      = 0.0                                                                         # U1
        self.M      = np.array([0.0, 0.0, 0.0]).reshape(3,1)                                      # U2, U3, U4 respectively
