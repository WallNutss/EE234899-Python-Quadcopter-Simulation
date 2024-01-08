import numpy as np
from math import pi, sin, cos, tan, atan
from lib.rotation import RPY2XYZ, SpecialR, D2R, R2D
import keyboard

# Import Controller
from lib.Controller import PID, antiWindup, SMC, LQR

# Import funtions
from lib.Controller import tanh

# Import Turbulance Wind
from windModel.dydrenwind import DydrenWind

class quadcopter:
    def __init__(self, Ts = 1.0/50.0, experiment='Default', **kwargs):
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
        self.kad    = 0.004609          # kgm^2/rad         , Aerodynamical drag coefficient
        self.Inert  = np.array([self.Ixx,         0,         0, \
                                        0, self.Iyy,         0, \
                                        0,        0,  self.Izz]).reshape(3,3)

        # Motor Params (Speed, rpm, etc)
        self.max_motor_speed = 2200.0   # rpm , max motor speed rotation per minute
        self.min_motor_speed = 0.0      # rpm , min motor speed rotation per minute
        self.u1_max          = 19.57    # N
        self.u1_min          = 0.0      # N
        self.u2_max          = 2.32     # Nm
        self.u2_min          = -2.32    # Nm
        self.u3_max          = 2.32     # Nm
        self.u3_min          = -2.32    # Nm
        self.u4_max          = 0.73     # Nm
        self.u4_min          = -0.73    # Nm

        # States of the Drone (Position, Orientation, Attitude)
        self.Time   = 0.0                                                                           # seconds , Initial Time Simulation
        self.initPosition = np.array([0.0, 0.0, 0.0]).reshape(3,1)                                  # X,Y,Z , Position Initial in Global Frame
        self.Ts     = Ts                # s     , Time sampling of the simulation
        self.state  = np.array([self.initPosition[0][0], self.initPosition[1][0], self.initPosition[2][0],\
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(12,1)       # X,Y,Z, dX,dY,dZ, 𝜑,𝜃,𝜓, p,q,r
                                                                                                   # First 6 State in Inertial Frame where the last of them in Body Frame
        self.r      = self.state[0:3]
        self.dr     = self.state[3:6]
        '''
        state is the thing we measured. X,Y,Z means it on global frame where x,y,z is on the body frame
        '''
        self.dstate = np.array([0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape(12,1)              # The derivative of the state , dX,dY,Z, ddX,ddY,ddZ, d𝜑,d𝜃,d𝜓, dp,dq,dr
        self.U      = np.array([self.u1_min, self.u2_min, self.u3_min, self.u4_min]).reshape(4,1)   # Control Inputs for the dynamics mathematical model after
        self.Thrust = self.U[0]                                                                     # U1
        self.M      = self.U[1:4]                                                                   # U2, U3, U4 respectively

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

        # For SMC

        # For Logs Purpose
        self.timeLogs  = np.array([0.0])
        self.errorxyz  = np.array([0.0, 0.0, 0.0])
        self.angleLogs = np.array([0.0, 0.0, 0.0])
        self.posLogs   = np.array([0.0, 0.0, 0.0])
        self.ULogs     = np.array([0.0, 0.0, 0.0, 0.0])
        self.SlidingSurface = np.array([0.0, 0.0, 0.0])

        # Wind Turbulance for Uncentainties and etc
        self.wind = DydrenWind(height=6.076, airspeed=2, turbulancelvl=15).Model()
        self.DISTURBANCE = kwargs['DISTURBANCE']
        self.iter = 0 # it's for local counting for inserting wind disturbance value
        self.PID = True
        self.experiment = experiment
        self.controller = kwargs['controller']
        self.smctype = kwargs['smctype']

        # For Sliding Mode Control
        self.lamda = []
        self.lamda2 = []

    def DynamicSolver(self):
        """
        Perform a calculation based on Quadcopter Dynamic Mathematical Model
        Args
            self.state[0:3]  = X,Y,Z {EF}
            self.state[3:6]  = dX,dY,dZ {EF}
            self.state[6:9]  = 𝜑,𝜃,𝜓 {EF} ~ {BF}
            self.state[9:12] = p,q,r {BF}
        Return
            Double Derivative Position and Attitude of Quadcopter in Global Frame
        """
        # Getting the rotation matrix first
        bRe = RPY2XYZ(self.state[6:9])
        R   = bRe.transpose()

        #Derivative State 1 to 3, the velocity, because past state define the future state,in this function all the disturbance wether its internal of external is put down here
        if self.DISTURBANCE:
            if self.Time > 0:
                try:
                    self.dstate[0:3] = self.state[3:6] + self.wind[self.iter,:].reshape(3,1)
                except:
                    self.dstate[0:3] = self.state[3:6]
                #self.dstate[0:3] = self.state[3:6] + np.array([2.0, 2.0,2.0]).reshape(3,1)
            else:
                self.dstate[0:3] = self.state[3:6]
        else:
            self.dstate[0:3] = self.state[3:6]

        # Calculation of Dynamics Translation Motion of the Drone which is the Acceleration of the Model
        self.dstate[3:6] = (- self.g * np.array([0.0, 0.0, 1.0]).reshape(3,1)) + \
                           ( (1/self.m) * np.matmul(R, self.Thrust * np.array([0.0, 0.0, 1.0]).reshape(3,1))) - \
                            ((1/self.m) * np.array([self.ktd * self.dstate[0], self.ktd * self.dstate[1], self.ktd * self.dstate[2]]).reshape(3,1))

        # Calculation from the Special Transfer Matrix for Angular Rates (This make conversion from Body Frame to Inertial Frame)
        self.dstate[6:9] = np.matmul(SpecialR(self.state[6:9]), self.state[9:12])

        # Calculation of Rotational Motion of the Drone (Body Frame) which is the Acceleration of the Model
        Iw = np.matmul(self.Inert, self.state[9:12])
        self.dstate[9:12] = np.matmul((np.linalg.inv(self.Inert)), (self.M -\
                                                                 np.cross(self.state[9:12][:,0], Iw[:,0]).reshape(3,1) +\
                                                                 np.array([self.kad * (self.dstate[6]), self.kad * (self.dstate[7]), self.kad * (self.dstate[8])]).reshape(3,1)))

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
        self.iter = self.iter + 1
        # Updating Simulation time
        self.Time = self.Time + self.Ts

        # Calling for dynamic mathematical model calculation
        self.DynamicSolver()

        # Integration from derivative state so we can get Original State // Dead Reckoning // Euler Integration Method Look it up
        self.state  = self.state + (self.dstate * self.Ts)
        # Update and Assigning Each Invidual State to Duplicate Variable
        # Position {EF}
        self.r      = self.state[0:3]
        self.dr     = self.state[3:6]
        # Orientation
        self.angles = self.state[6:9]
        self.w      = self.state[9:12]

        # print('Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°' % (R2D(self.state[6]), R2D(self.state[7]), R2D(self.state[8])))

        # Update Position Error, no other way now
        self.x_err = self.x_des - self.r[0]
        self.y_err = self.y_des - self.r[1]
        self.z_err = self.z_des - self.r[2]

        # Logs & History
        self.timeLogs  = np.append(self.timeLogs, self.Time)
        self.angleLogs = np.vstack((self.angleLogs, np.array([R2D(self.angles[0][0]),R2D(self.angles[1][0]) ,R2D(self.angles[2][0])])))
        self.posLogs   = np.vstack((self.posLogs, self.r.flatten()))
        self.ULogs     = np.vstack((self.ULogs, self.U.flatten()))
        self.errorxyz = np.vstack((self.errorxyz, np.array([self.x_err, self.y_err, self.z_err]).flatten()))

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

        # Make them in one array
        pos_error = np.array([self.x_err, self.y_err, self.z_err]).reshape(3,1)
        pos_error_dot = np.array([(self.x_err - self.x_err_prev)/self.Ts, (self.y_err - self.y_err_prev)/self.Ts, (self.z_err - self.z_err_prev)/self.Ts]).reshape(3,1)
        integral_pos_error = np.array([self.x_err_sum, self.y_err_sum, self.z_err_sum]).reshape(3,1)

        # Feedback Linearization Start / Is this even Feedback Linearization lmao
        ux = PID(self.Ts, self.x_err, self.x_err_prev, self.x_err_sum, gains=np.array([-0.6, 0.0, -0.25]))
        uy = PID(self.Ts, self.y_err, self.y_err_prev, self.y_err_sum, gains=np.array([-0.6, 0.0,-0.25]))
        uz = PID(self.Ts, self.z_err, self.z_err_prev, self.z_err_sum, gains=np.array([-0.6, 0.0,-0.5]))

        # Control to Desired Angle Conversion
        # ux, uy, uz equals to double derivative of the error itself
        # Desired Phi
        a = (ux)/(uz + self.g)
        b = (uy)/(uz + self.g)
        c = cos(self.psi_des)
        d = sin(self.psi_des)

        # I know this will fucked me up later, but I still don't know the cause why
        # When I put - sign in all desired angles, it will work
        # EDIT : Fuck Fuck Fuck, When you put Ux and Uy on the negative side, it all make sense when I dont put - sign in angles desired
        # MORE EDIT : I Forgot why I didnt put the - sign in a,b. It does make sense weeks ago, oh well tehe, guess keep it inside. As long it works I don't mind know, the due is in two weeks again :)
        if self.psi_des < pi/4 or self.psi_des > 3*pi/4:
            self.phi_des = atan((cos(self.state[7])*(tan(self.state[7])*d - b))/(c))
        else:
            self.phi_des = atan((cos(self.state[7])*(a - tan(self.state[7])*c))/(d))

        #Desired Theta
        self.theta_des = atan(a*c + b*d)
        #self.theta_des =  atan(((-ux*cos(self.state[8]))/(-uz+self.g))+\
        #                    ((-uy*sin(self.state[8]))/(-uz+self.g)))

        # Anti Windup
        self.phi_des = antiWindup(self.phi_des, D2R(-45), D2R(45))
        self.theta_des = antiWindup(self.theta_des, D2R(-45), D2R(45))


        self.U[0] = (self.m * (-uz + self.g))/(cos(self.state[6])*cos(self.state[7]))
        # self.U[0] = (self.m * (-uz + self.g)) --> LOL, this algorithm is also working. At this point Idk what is right and what is wrong anymore
    
        # Updating Error Value
        self.x_err_prev = self.x_err
        self.x_err_sum = self.x_err_sum + self.x_err

        self.y_err_prev = self.y_err
        self.y_err_sum = self.y_err_sum + self.y_err

        self.z_err_prev = self.z_err
        self.z_err_sum = self.z_err_sum + self.z_err


    def attitudeController(self):
        """
        Attitude Controller Calculation

        Parameters
        ----------
        **kwargs: Additional parameters for SMC Type Controller
            `1` controller (`int`) , type of controller we want. `0` == PID , `1` == SMC\n
            `2` smctype (`int`), type of SMC we want. `1` == SMC-Sign , `2` == SMC-Sat, `3` == SMC-Tanh , `4` == ISMC-Tanh, `5` == Tripathi Reference ,
        
        Return
        ----------
        Nothing, it just a method calculation. Return is in form on Re-Iniliazation of Self Class
        """
        #Getting the measurement first
        phi = self.state[6]
        theta = self.state[7]
        psi = self.state[8]

        # Getting the error from reference - measurement
        self.phi_err = self.phi_des - phi
        self.theta_err = self.theta_des - theta
        self.psi_err = self.psi_des - psi

        # Combining them in one array for log purpose
        angle_error = np.array([self.phi_err, self.theta_err, self.psi_err]).reshape(3,1)
        angle_error_dot = np.array([(self.phi_err - self.phi_err_prev)/self.Ts, (self.theta_err - self.theta_err_prev)/self.Ts, (self.psi_err - self.psi_err_prev)/self.Ts]).reshape(3,1)
        integral_angle_error = np.array([self.phi_err_sum, self.theta_err_sum, self.psi_err_sum]).reshape(3,1)

        # Calculation from the Special Transfer Matrix for Angular Rates (This make conversion from Body Frame to Inertial Frame), this is angulare rates on Inertial Frame Perspective
        angles_dot = np.matmul(SpecialR(self.state[6:9]), self.state[9:12]) # Kecepatan perubahan sudut di euler angles ({EF})

        if self.controller == 0: # PID Controller
            # U2
            self.U[1] = PID(self.Ts, self.phi_err, self.phi_err_prev, self.phi_err_sum, gains=np.array([1.3, 0.03, 0.3]))
            # U3
            self.U[2] = PID(self.Ts, self.theta_err, self.theta_err_prev, self.theta_err_sum, gains=np.array([1.3, 0.03, 0.3]))
            # U4
            self.U[3] = PID(self.Ts, self.psi_err, self.psi_err_prev, self.psi_err_sum, gains=np.array([1.3, 0.03, 0.3]))

        elif self.controller == 1: # Sliding Mode Controller
            # Defining Control Equation for SMC Controller
            control, slidingsurface, smctype = SMC(self.Inert, angle_error, angles_dot, angle_error_dot, integral_angle_error, control=self.smctype)
            # U2_Equation
            self.U[1] = control[0]
            # U3_Equation
            self.U[2] = control[1]
            # U4_Equation
            self.U[3] = control[2]

            # For logs and History
            self.SlidingSurface = np.vstack((self.SlidingSurface, slidingsurface.flatten()))

        # Anti Windup
        self.U[0] = antiWindup(self.U[0], self.u1_min, self.u1_max)
        self.U[1] = antiWindup(self.U[1], self.u2_min, self.u2_max)
        self.U[2] = antiWindup(self.U[2], self.u3_min, self.u3_max)
        self.U[3] = antiWindup(self.U[3], self.u4_min, self.u4_max)

        # Updating Error Value
        self.phi_err_prev = self.phi_err
        self.phi_err_sum = self.phi_err_sum + self.phi_err

        self.theta_err_prev = self.theta_err
        self.theta_err_sum = self.theta_err_sum + self.theta_err

        self.psi_err_prev = self.psi_err
        self.psi_err_sum = self.psi_err_sum + self.psi_err

        # Updating Error Desired Value
        self.phi_des_prev = self.phi_des
        self.theta_des_prev = self.theta_des
        self.psi_des_prev = self.psi_des

        # Updating Control Blocks
        self.Thrust = self.U[0]
        self.M = self.U[1:4]
