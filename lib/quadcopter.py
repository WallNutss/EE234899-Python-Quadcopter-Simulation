import numpy as np
from math import pi, sin, cos, tan, atan
from lib.rotation import RPY2XYZ, SpecialR, D2R, R2D

# Import Controller
from lib.controller import PID

class quadcopter:
    def __init__(self, Ts = 1.0/50.0):

        # Tello Params
        self.g      = 9.81              # m/s^2             , gravity acceleration
        self.m      = 0.08              # kg                , mass of the drone
        self.l      = 0.06              # m                 , length of the each arm
        self.Ixx    = 0.0068            # kgm^2             , Symetrical Inertia matrix at x-axis
        self.Iyy    = 0.0068            # kgm^2             , Symetrical Inertia matrix at y-axis
        self.Izz    = 0.0131            # kgm^2             , Symetrical Inertia matrix at z-axis
        self.Ir     = 0.0000495         # kgm^2             , The moment of inertia of each rotor
        self.b      = 0.00000566        # kgm/ (rad)^2      , Thrust Constant
        self.d      = 0.0000000580644   # kgm^2/ (rad)^2    , Drag factor
        self.ktd    = 0.03365           # kg/s              , Translational drag coefficient
        self.Inert  = np.array([self.Ixx,         0,         0, \
                                        0, self.Iyy,         0, \
                                        0,        0,  self.Izz]).reshape(3,3)

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
        self.Time   = 0.0               # s     , Initial Time Simulation
        self.initPosition = np.array([0.3, 0.6, 0.2]).reshape(3,1)
        self.Ts     = Ts                # s     , Time sampling of the simulation
        self.state  = np.array([self.initPosition[0][0], self.initPosition[1][0], self.initPosition[2][0],\
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(12,1)       # X,Y,Z, dX,dY,dZ, 𝜑,𝜃,𝜓, p,q,r
                                                                                                                 # First 6 State in Inertial Frame where the last of them in Body Frame
        self.r      = self.state[0:3]
        self.dr     = self.state[3:6]
        '''
        state is the thing we measured. X,Y,Z means it on global frame where x,y,z is on the body frame
        '''
        self.dstate = np.array([0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape(12,1)       # The derivative of the state , dX,dY,Z, ddX,ddY,ddZ, d𝜑,d𝜃,d𝜓, dp,dq,dr
        self.U      = np.array([self.u1_min, self.u2_min, self.u3_min, self.u4_min]).reshape(4,1)                             # Control Inputs for the dynamics mathematical model after
        self.Thrust = self.U[0]                                                                                        # U1
        self.M      = self.U[1:4]                                                   # U2, U3, U4 respectively

        # States of the Drone Orientation
        self.angles = self.state[6:9]                                        # 𝜑,𝜃,𝜓 , the working angle in body frame
        self.w      = self.state[9:12]                                       # p,q,r , The angular velocity of the drone respective to the Body Frame

        # States of the Error we need for Attitude Controller
        self.phi_des = 0.0
        self.theta_des = 0.0
        self.psi_des = 0.0

        self.phi_err = 0.0
        self.theta_err = 0.0
        self.psi_err = 0.0
        
        self.phi_err_prev = 0.0
        self.theta_err_prev = 0.0
        self.psi_err_prev = 0.0
        
        self.phi_err_sum = 0.0
        self.theta_err_sum = 0.0
        self.psi_err_sum = 0.0
    
        # States of the Error we need for Position Controller
        self.x_des = 0.0
        self.y_des = 0.0
        self.z_des = 0.0

        self.x_err = 0.0
        self.y_err = 0.0
        self.z_err = 0.0
        
        self.x_err_prev = 0.0
        self.y_err_prev = 0.0
        self.z_err_prev = 0.0
        
        self.x_err_sum = 0.0
        self.y_err_sum = 0.0
        self.z_err_sum = 0.0

    def DynamicSolver(self):
        """
        Perform a calculation based on Quadcopter Dynamic Mathematical Model
        Args
            self.state[0:3]  = X,Y,Z
            self.state[3:6]  = dX,dY,dZ
            self.state[6:9]  = 𝜑,𝜃,𝜓
            self.state[9:12] = p,q,r
        Return
            Double Derivative Position and Attitude of Quadcopter in Global Frame 
        """
        # Getting the rotation matrix first
        bRe = RPY2XYZ(self.state[6:9])
        R   = bRe.transpose()
        
        # Derivative State 1 to 3, the velocity, because past state define the future state
        self.dstate[0:3] = self.state[3:6]

        # Calculation of Dynamics Translation Motion of the Drone which is the Acceleration of the Model
        self.dstate[3:6] = (-self.g * np.array([0.0, 0.0, 1.0]).reshape(3,1)) + \
                           ( (1/self.m) * np.dot(R, self.Thrust * np.array([0.0, 0.0, 1.0]).reshape(3,1)))- \
                           ((1/self.m) * np.array([self.ktd, self.ktd, self.ktd]).reshape(3,1))

        # Calculation from the Special Transfer Matrix for Angular Rates (Inertial Frame
        self.dstate[6:9] = np.dot(SpecialR(self.state[6:9]), self.state[9:12])
        
        # Calculation of Rotational Motion of the Drone (Body Frame) which is the Acceleration of the Model
        Iw = np.dot(self.Inert, self.state[9:12])
        self.dstate[9:12] = np.dot((np.linalg.inv(self.Inert)), (self.M - np.cross(self.state[9:12][:,0], Iw[:,0]).reshape(3,1)))

        # Immediately Integral of dstate[3:6] which is the result of acceleration in Integral Frame to get Velocity also in {EF}
        # self.dstate[0:3] = self.dstate[0:3] + self.dstate[3:6] * self.Ts

    def fetchUpdate(self):
        return self.state
    
    def updateState(self):
        """
        Perform a update pose based on calculation we had from Dynamic Solver. It's there but not in the shape we want
        Args
            self.dstate[0:3]  = dX,dY,dZ
            self.dstate[3:6]  = ddX,ddY,ddZ
            self.dstate[6:9]  = d𝜑,d𝜃,d𝜓
            self.dstate[9:12] = dp,dq,dr
        Return
            Position 
        """
        # Updating Simulation time
        self.Time = self.Time + self.Ts

        # Calling for dynamic mathematical model calculation    
        self.DynamicSolver()

        # Integration from derivative state so we can get Original State
        self.state = self.state + (self.dstate * self.Ts)
        # Update and Assigning Each Invidual State to Duplicate Variable
        # Position {EF}
        self.r      = self.state[0:3]
        self.dr     = self.state[3:6]
        # Orientation
        self.angles = self.state[6:9]
        self.w      = self.state[9:12]

    def attitudeController(self, Reference):
        #Getting the measurement first
        phi = self.state[6]
        theta = self.state[7]
        psi = self.state[8]

        self.phi_des = Reference[0]
        self.theta_des = Reference[1]
        #self.psi_des = Reference[2]
        print(self.psi_des)

        # Getting the error from reference - measurement
        self.phi_err = self.phi_des - phi
        self.theta_err = self.theta_des - theta
        self.psi_err = self.psi_des - psi

        # U2
        self.U[1] = PID(self.Ts, self.phi_err, self.phi_err_prev, self.phi_err_sum, gains=np.array([0.3, 0.02, 0.2]))
        # U3
        self.U[2] = PID(self.Ts, self.theta_err, self.theta_err_prev, self.theta_err_sum, gains=np.array([0.8, 0.03, 0.3]))
        # U4
        self.U[3] = PID(self.Ts, self.psi_err, self.psi_err_prev, self.psi_err_sum, gains=np.array([0.2, 0.01, 0.4]))

        # Updating Error Value
        self.phi_err_prev = self.phi_err
        self.phi_err_sum = self.phi_err_sum + self.phi_err

        self.theta_err_prev = self.theta_err
        self.theta_err_sum = self.theta_err_sum + self.theta_err

        self.psi_err_prev = self.psi_err
        self.psi_err_sum = self.psi_err_sum + self.psi_err   

        # Updating Control Blocks
        self.U[0] = (self.m * self.g)*1.2
        self.Thrust = self.U[0]
        self.M = self.U[1:4]

    def positionController(self, Reference):
        #Getting the measurement first, this is on Inertial Frame {EF}
        x_pos = self.state[0]
        y_pos = self.state[1]
        z_pos = self.state[2]

        #Getting the Reference,also on Inertial Frame {EF}
        self.x_des = Reference[0]
        self.y_des = Reference[1]
        self.z_des = Reference[2]

        # Getting the error from reference - measurement
        self.x_err = self.x_des - x_pos
        self.y_err = self.y_des - y_pos
        self.z_err = self.z_des - z_pos

        # Feedback Linearization Start
        ux = PID(self.Ts, self.x_err, self.x_err_prev, self.x_err_sum, gains=np.array([-1.0, 0.0,-2.0]))
        uy = PID(self.Ts, self.y_err, self.y_err_prev, self.y_err_sum, gains=np.array([-1.0, 0.0,-2.0]))
        uz = PID(self.Ts, self.z_err, self.z_err_prev, self.z_err_sum, gains=np.array([-1.0, 0.0,-2.0]))

        # Control to Desired Angle Conversion
        # ux, uy, uz equals to double derivative of the error itself
        # Desired Phi
        a = (-ux)/(-uz + self.g)
        b = (-uy)/(-uz + self.g)
        c = cos(self.psi_des)
        d = sin(self.psi_des)

        if self.psi_des < pi/4 or self.psi_des > pi/4:
            self.phi_des = atan((cos(self.state[7])*(tan(self.state[7])*d - b))/(c))
        else:
            self.phi_des = atan((cos(self.state[7])*(a - tan(self.state[7])*c))/(d))
        
        #Desired Theta
        self.theta = atan(((-ux*cos(self.state[8]))/(-uz+self.g))+\
                            ((-uy*sin(self.state[8]))/(-uz+self.g)))
        
        self.U[0] = (self.m * (-uz + self.g))/(cos(self.state[6])*cos(self.state[7]))

        # Updating Error Value
        self.x_err_prev = self.x_err
        self.x_err_sum = self.x_err_sum + self.x_err

        self.y_err_prev = self.y_err
        self.y_err_sum = self.y_err_sum + self.y_err

        self.z_err_prev = self.z_err
        self.z_err_sum = self.z_err_sum + self.z_err  
        



        