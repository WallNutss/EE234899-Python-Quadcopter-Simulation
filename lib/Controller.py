import numpy as np
import math

def sat(v):
    v = np.copy(v)
    v[np.abs(v) > 0.1] = np.sign(v[np.abs(v) > 0.1])
    return v

def tanh(x):
    ans = (math.e**x - math.e**-x)/(math.e**x + math.e**-x)
    return ans

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

def SMC(sliding, inertia, lamda, lamda2, angle_error, angles_dot, angle_error_dot, integral_angle_error):
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
    K = np.array([3.5,      0,    0, \
                   0,      3.5,    0, \
                   0,      0,    3.5]).reshape(3,3)
    K_2 = np.array([1,      0,    0, \
                     0,      1,    0, \
                     0,      0,    1]).reshape(3,3)   
    
    a1 = (inertia[1][1] - inertia[2][2])/inertia[0][0]
    b1 = 1/inertia[0][0]

    a2 = (inertia[2][2] - inertia[0][0])/inertia[1][1]
    b2 = 1/inertia[1][1]

    a3 = (inertia[0][0] - inertia[1][1])/inertia[2][2]
    b3 = 1/inertia[2][2]

    # Roll Control
    # The first trial of the SMC Control
    #U_2 = (1/b1)*( K[0][0]*np.sign(sliding[0]) - a1*angles_dot[1]*angles_dot[2] + lamda[0][0]*angle_error_dot[0] )
    # The second trial of the SMC control
    #U_2 = (1/b1)*( K[0][0]*sat(sliding[0]) - a1*angles_dot[1]*angles_dot[2] + lamda[0][0]*angle_error_dot[0] )
    # The third trial of the SMC control
    #U_2 = (1/b1)*( K[0][0]*tanh(sliding[0]) - a1*angles_dot[1]*angles_dot[2] + lamda[0][0]*angle_error_dot[0] )

    # The fourh part of the smc control
    U_2 = (1/b1)*( K[0][0]*tanh(sliding[0]) - a1*angles_dot[1]*angles_dot[2] + lamda[0][0]*angle_error_dot[0] + lamda2[0][0]*angle_error[0])

    #U_2 = (1/b1)*( K[0][0]*sat(sliding[0]) - a1*angles_dot[1]*angles_dot[2] + lamda[0][0]*angle_error_dot[0] )
    #U_2 = (1/b1)*( K[0][0]*sat(sliding[0]) + K_2[0][0]*sliding[0] - a1*angles_dot[1]*angles_dot[2] - lamda[0][0]*angle_error_dot[0] )
    #U_2 = (1/b1)*( K[0][0]*np.sign(sliding[0]) + K_2[0][0]*sliding[0] - a1*angles_dot[1]*angles_dot[2] + lamda[0][0]*angle_error_dot[0] )

    # Pitch Control
    # The first trial of the SMC Control
    #U_3 = (1/b2)*( K[1][1]*np.sign(sliding[1]) - a2*angles_dot[0]*angles_dot[2] + lamda[1][1]*angle_error_dot[1] )
    # The second trial of the SMC Control
    #U_3 = (1/b2)*( K[1][1]*sat(sliding[1]) - a2*angles_dot[0]*angles_dot[2] + lamda[1][1]*angle_error_dot[1] )
    # The third trial of the SMC control
    #U_3 = (1/b2)*( K[1][1]*tanh(sliding[1]) - a2*angles_dot[0]*angles_dot[2] + lamda[1][1]*angle_error_dot[1] )  

    # The fourh part of the SMC Contrl
    U_3 = (1/b2)*( K[1][1]*tanh(sliding[1]) - a2*angles_dot[0]*angles_dot[2] + lamda[1][1]*angle_error_dot[1] + lamda2[1][1]*angle_error[1])  

    #U_3 = (1/b2)*( K[1][1]*sat(sliding[1]) - a2*angles_dot[0]*angles_dot[2] + lamda[1][1]*angle_error_dot[1] )
    #U_3 = (1/b2)*( K[1][1]*sat(sliding[1]) + K_2[1][1]*sliding[1] - a2*angles_dot[0]*angles_dot[2] - lamda[1][1]*angle_error_dot[1] )
    #U_3 = (1/b2)*( K[1][1]*np.sign(sliding[1]) + K_2[1][1]*sliding[1] - a2*angles_dot[0]*angles_dot[2] + lamda[1][1]*angle_error_dot[1] )

    # Yaw Control
    # The first trial of the SMC Control
    #U_4 = (1/b3)*( K[2][2]*np.sign(sliding[2]) - a3*angles_dot[0]*angles_dot[1] + lamda[2][2]*angle_error_dot[2] )
    # The second trial of the SMC Control
    #U_4 = (1/b3)*( K[2][2]*sat(sliding[2]) - a3*angles_dot[0]*angles_dot[1] + lamda[2][2]*angle_error_dot[2] )
    # The third trial of the SMC control    
    #U_4 = (1/b3)*( K[2][2]*tanh(sliding[2]) - a3*angles_dot[0]*angles_dot[1] + lamda[2][2]*angle_error_dot[2] )

    # The fourth part of the SMC Control
    U_4 = (1/b3)*( K[2][2]*tanh(sliding[2]) - a3*angles_dot[0]*angles_dot[1] + lamda[2][2]*angle_error_dot[2] + lamda2[2][2]*angle_error[2])

    #U_4 = (1/b3)*( K[2][2]*sat(sliding[2]) - a3*angles_dot[0]*angles_dot[1] + lamda[2][2]*angle_error_dot[2] )
    #U_4 = (1/b3)*( K[2][2]*sat(sliding[2]) + K_2[2][2]*sliding[2] - a3*angles_dot[0]*angles_dot[1] - lamda[2][2]*angle_error_dot[2] )
    #U_4 = (1/b3)*( K[2][2]*np.sign(sliding[2]) + K_2[2][2]*sliding[2] - a3*angles_dot[0]*angles_dot[1] + lamda[2][2]*angle_error_dot[2] )


    '''
    Messsage corner

    Haha goblok, sekarang pas b1nya kubaikin sesuai dinamika punyaku, sekarang fungsi saturasinya yang nggak working, tapi signnya working
    Hadeh

    What's working
    1. Saat u_switch = -Ksgn(s), bisa bekerja dengan baik jika b1,b2,b3 penyebutnya == 1 bukan l
    2. Saat u_switch = -Ksat(s), bisa bekejra dengan baik jika b1,b2,b3 penyebutnya == l bukan 1

    '''
    U_SMC = np.array([U_2,U_3,U_4]).reshape(3,1)
    return U_SMC

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