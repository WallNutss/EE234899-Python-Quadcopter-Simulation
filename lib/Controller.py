import numpy as np
import math

def sat(v):
    v = np.copy(v)
    v[np.abs(v) > 0.1] = np.sign(v[np.abs(v) > 0.1])
    return v

def tanh(x):
    ans = (math.e**x - math.e**-x)/(math.e**x + math.e**-x)
    return ans

def sobolevaTanh(x):
    if x > 0:
        return x
    elif x < 0:
        return 0.01*x
    else:
        return 0

def PID(Ts, err, err_prev, err_sum, gains):
    """
    Performing a Proportional-Integral-Derivative (PID) controller
    
    Args
        Ts          = Time Sampling of the simulation\n
        err         = The error difference between reference and measurement value\n
        err_prev    = Error from previous state of the simulation\n
        err_sum     = The sum of from all the error\n
        gains       = Gains for Proportional-Integral-Derivative controller\n
    Return 
        Control Input for the system
    """
    return (gains[0] * err) + (err_sum * gains[1]) + (gains[2] * (err - err_prev)/Ts)

'''
def SMC(sliding, inertia, angles_dot, lamda, angle_error_dot):
    K = np.array([0.19,      0,    0, \
                    0,    0.19,    0, \
                    0,      0,  0.19]).reshape(3,3)
    # Let him cook
    inert_inv = np.linalg.inv(inertia)

    # Part A, the Control Equation for attitude
    Iw   = np.dot(inertia, angles_dot) # For the I.[p,q,r]
    Invw = np.dot(-inert_inv, angles_dot) # For the I^-1.[p,q,r]

    Inertia_F = np.cross(Invw[:,0], Iw[:,0]).reshape(3,1)
    U_eq = Inertia_F  + lamda.dot(angle_error_dot)
    #U_eq = inert.dot(Inertia_F - lamda.dot(anglevelocity))

    sliding[0] = sat(sliding[0])
    sliding[1] = sat(sliding[1])
    sliding[2] = sat(sliding[2])

    u_smc = (U_eq - K.dot(sliding))
    U_SMC = inertia.dot(u_smc)

    print(U_SMC)
    print("-------------------------")
    return U_SMC
'''

def SMC(inertia, angle_error, angles_dot, angle_error_dot, integral_angle_error, **kwargs):
    """
    Performing a custom (SMC) controller special for this simulation
    
    Args
        sliding                 = Sldiing surface at t simulation of each angle\n
        inertia                 = The inertia matriks of the Quadcopter\n
        lamda                   = Lambda gain parameter of the SMC Controller\n
        angles_dot              = The derivative of the angle\n
        angle_error_dot         = The derivative of the error of angle\n
        integral_angle_error    = The sum of from all the error\n
    Return 
        Control Input for the system
    """
    # Reference --> Tripathi(2015), Design of Sliding Mode and Backstepping Controllers for a Quadcopter, Kiriman Tuhan Ya Allah, sisa baikin sama ngertiin kinematika kok bisa working nih
    control = kwargs['control']
    
    K = np.array([3.5,      0,    0, \
                   0,      3.5,    0, \
                   0,      0,    3.5]).reshape(3,3)
    K_2 = np.array([1,      0,    0, \
                     0,      1,    0, \
                     0,      0,    1]).reshape(3,3)   
    
    lamda = np.array([  4.5,      0,    0, \
                                 0,      4.5,    0, \
                                 0,      0,    4.5]).reshape(3,3)
    lamda2 = np.array([ 0.1,      0,    0, \
                                 0,     0.1,    0, \
                                 0,      0,    0.1]).reshape(3,3)
    
    a1 = (inertia[1][1] - inertia[2][2])/inertia[0][0]
    b1 = 1/inertia[0][0]

    a2 = (inertia[2][2] - inertia[0][0])/inertia[1][1]
    b2 = 1/inertia[1][1]

    a3 = (inertia[0][0] - inertia[1][1])/inertia[2][2]
    b3 = 1/inertia[2][2]

    if control == 1: # Conventional SMC with Sign Switching Function
        slidingsurface = angle_error_dot + np.matmul(lamda, angle_error)
        # Roll Control
        U_2 = (1/b1)*( K[0][0]*np.sign(slidingsurface[0]) - a1*angles_dot[1]*angles_dot[2] + lamda[0][0]*angle_error_dot[0] )
        # Pitch Control
        U_3 = (1/b2)*( K[1][1]*np.sign(slidingsurface[1]) - a2*angles_dot[0]*angles_dot[2] + lamda[1][1]*angle_error_dot[1] )
        # Yaw Control
        U_4 = (1/b3)*( K[2][2]*np.sign(slidingsurface[2]) - a3*angles_dot[0]*angles_dot[1] + lamda[2][2]*angle_error_dot[2] )

        smctype = 'Conventional SMC with Sign Switching Function'

    elif control == 2: # Conventional SMC but with Saturation Switching Function
        slidingsurface = angle_error_dot + np.matmul(lamda, angle_error)
        # Roll Control
        U_2 = (1/b1)*( K[0][0]*sat(slidingsurface[0]) - a1*angles_dot[1]*angles_dot[2] + lamda[0][0]*angle_error_dot[0] )
        # Pitch Control
        U_3 = (1/b2)*( K[1][1]*sat(slidingsurface[1]) - a2*angles_dot[0]*angles_dot[2] + lamda[1][1]*angle_error_dot[1] )
        # Yaw Control
        U_4 = (1/b3)*( K[2][2]*sat(slidingsurface[2]) - a3*angles_dot[0]*angles_dot[1] + lamda[2][2]*angle_error_dot[2] )

        smctype = 'Conventional SMC but with Saturation Switching Function'

    elif control == 3: # Conventional SMC but with Tanh Switching Function
        slidingsurface = angle_error_dot + np.matmul(lamda, angle_error)
        # Roll Control
        U_2 = (1/b1)*( K[0][0]*tanh(slidingsurface[0]) - a1*angles_dot[1]*angles_dot[2] + lamda[0][0]*angle_error_dot[0] )
        # Pitch Control
        U_3 = (1/b2)*( K[1][1]*tanh(slidingsurface[1]) - a2*angles_dot[0]*angles_dot[2] + lamda[1][1]*angle_error_dot[1] )
        # Yaw Control
        U_4 = (1/b3)*( K[2][2]*tanh(slidingsurface[2]) - a3*angles_dot[0]*angles_dot[1] + lamda[2][2]*angle_error_dot[2] )

        smctype = 'Conventional SMC but with Tanh Switching Function'
        

    elif control == 4: # Integral Sliding Mode Control with Modified Tanh Switching Function
        slidingsurface = angle_error_dot + np.matmul(lamda, angle_error) +  np.matmul(lamda2, integral_angle_error)
        # Roll Control
        U_2 = (1/b1)*( K[0][0]*tanh(slidingsurface[0]) - a1*angles_dot[1]*angles_dot[2] + lamda[0][0]*angle_error_dot[0] + lamda2[0][0]*angle_error[0])
        # Pitch Control
        U_3 = (1/b2)*( K[1][1]*tanh(slidingsurface[1]) - a2*angles_dot[0]*angles_dot[2] + lamda[1][1]*angle_error_dot[1] + lamda2[1][1]*angle_error[1])
        # Yaw Control
        U_4 = (1/b3)*( K[2][2]*tanh(slidingsurface[2]) - a3*angles_dot[0]*angles_dot[1] + lamda[2][2]*angle_error_dot[2] + lamda2[2][2]*angle_error[2])

        smctype = 'Integral Sliding Mode Control with Modified Tanh Switching Function' 

    elif control == 5: # Control method from Adapthi GOD paper with modified adding sliding value to switching function
        slidingsurface = angle_error_dot + np.matmul(lamda, angle_error)
        # Roll Control
        U_2 = (1/b1)*( K[0][0]*sat(slidingsurface[0]) + K_2[0][0]*slidingsurface[0] - a1*angles_dot[1]*angles_dot[2] - lamda[0][0]*angle_error_dot[0] )
        # Pitch Control
        U_3 = (1/b2)*( K[1][1]*sat(slidingsurface[1]) + K_2[1][1]*slidingsurface[1] - a2*angles_dot[0]*angles_dot[2] - lamda[1][1]*angle_error_dot[1] )
        # Yaw Control
        U_4 = (1/b3)*( K[2][2]*sat(slidingsurface[2]) + K_2[2][2]*slidingsurface[2] - a3*angles_dot[0]*angles_dot[1] - lamda[2][2]*angle_error_dot[2] )

        smctype = 'Control method from Adapthi GOD paper with modified adding sliding value to switching function'

    '''
    Messsage corner

    Haha idiot, now when I fix the b1 according to my dynamics, now the saturation function is not working, but the sign function is working. Hadeh (Indonesia Moment Euy)

    What's working
    1. When u_switch = -Ksgn(s), it can work well if b1,b2,b3 denominators == 1, not l
    2. When u_switch = -Ksat(s), it works well if b1,b2,b3 denominator == l, not 1.

    '''
    U_SMC = np.array([U_2,U_3,U_4]).reshape(3,1)
    return U_SMC, slidingsurface, smctype

def LQR():
    # RN, for the base I go with linearization of the rates
    A = np.array([0, 0, 0, 1, 0, 0,\
                  0, 0, 0, 0, 1, 0,\
                  0, 0, 0, 0, 0, 1,\
                  0, 0, 0, 0, 0, 0,\
                  0, 0, 0, 0, 0, 0,\
                  0, 0, 0, 0, 0, 0,]).reshape(6,6)
    B = np.array([0,      0, 0,\
                  0,      0, 0,\
                  0,      0, 0,\
                  147.05, 0, 0,\
                  0, 147.05, 0,\
                  0, 0,  76.33]).reshape(6,3)
    Q = np.array([1, 0, 0, 0, 0, 0,\
                  0, 1, 0, 0, 0, 0,\
                  0, 0, 1, 0, 0, 0,\
                  0, 0, 0, 1, 0, 0,\
                  0, 0, 0, 0, 1, 0,\
                  0, 0, 0, 0, 0, 1,]).reshape(6,6)
    R = 0.5

    K = lqr(A,B,Q,R)
    return K

def antiWindup(x, x_min, x_max):
    """
    Perform Anti-Windup for any system that make system fuck up
    
    Args
        x     : Value need to be re-evaluated
        x_min : Minimum boundary
        x_max : Maximum boundary
    Return
        More stable x value
    """
    if x > x_max:
        return x_max
    elif x < x_min:
        return x_min
    else:
        return x