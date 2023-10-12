import numpy as np
import math

def sat(v):
    v = np.copy(v)
    v[np.abs(v) > 0.45] = np.sign(v[np.abs(v) > 0.45])
    return v

def PID(Ts, err, err_prev, err_sum, gains):
    """
    Performing a Proportional-Integral-Derivative (PID) controller
    
    Args
        Ts          = Time Sampling of the simulation
        err         = The error difference between reference and measurement value
        err_prev    = Error from previous state of the simulation
        err_sum     = The sum of from all the error
        gains       = Gains for Proportional-Integral-Derivative controller
    Return 
        Control Input for the system
    """
    return (gains[0] * err) + (err_sum * gains[1]) + (gains[2] * (err - err_prev)/Ts)

# def SMC(sliding, inert, anglevelocity, lamda, prevAngle):
#     K = np.array([0.19,      0,    0, \
#                     0,    0.19,    0, \
#                     0,      0,  0.01]).reshape(3,3)
#     # Let him cook
#     inert_inv = np.linalg.inv(inert)

#     # Part A, the Control Equation for attitude
#     Iw   = np.dot(inert, anglevelocity) # For the I.[p,q,r]
#     Invw = np.dot(inert_inv, anglevelocity) # For the I^-1.[p,q,r]

#     Inertia_F = np.cross(Invw[:,0], Iw[:,0]).reshape(3,1)
#     U_eq = inert.dot(Inertia_F - inert_inv.dot(0.004609 * anglevelocity) - lamda.dot(anglevelocity))
#     #U_eq = inert.dot(Inertia_F - lamda.dot(anglevelocity))

#     sliding[0] = sat(sliding[0])
#     sliding[1] = sat(sliding[1])
#     sliding[2] = sat(sliding[2])

#     U_SMC = U_eq - K.dot(sliding)

#     print(U_SMC)
#     print("-------------------------")
#     return U_SMC

def SMC(sliding, inertia, angles_dot, lamda, angle_error_dot):
    # Reference --> Tripathi(2015), Design of Sliding Mode and Backstepping Controllers for a Quadcopter, Kiriman Tuhan Ya Allah, sisa baikin sama ngertiin kinematika kok bisa working nih
    K = np.array([0.7,      0,    0, \
                   0,      0.7,    0, \
                   0,      0,    0.7]).reshape(3,3)
    K_2 = np.array([0.8,      0,    0, \
                     0,      0.8,    0, \
                     0,      0,    0.8]).reshape(3,3)   
    a1 = (inertia[1][1] - inertia[2][2])/inertia[0][0]
    b1 = 0.06/inertia[0][0]

    a2 = (inertia[2][2] - inertia[0][0])/inertia[1][1]
    b2 = 0.06/inertia[1][1]

    a3 = (inertia[0][0] - inertia[1][1])/inertia[2][2]
    b3 = 0.06/inertia[2][2]

    # Roll Control
    U_2 = (1/b1)*( K[0][0]*sat(sliding[0]) + K_2[0][0]*sliding[0] - a1*angles_dot[1]*angles_dot[2] + lamda[0][0]*angle_error_dot[0] )
    # Pitch Control
    U_3 = (1/b2)*( K[1][1]*sat(sliding[1]) + K_2[1][1]*sliding[1] - a2*angles_dot[0]*angles_dot[2] + lamda[1][1]*angle_error_dot[1] )
    # Yaw Control
    U_4 = (1/b3)*( K[2][2]*sat(sliding[2]) + K_2[2][2]*sliding[2] - a3*angles_dot[0]*angles_dot[1] + lamda[2][2]*angle_error_dot[2] )

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
    B = np.array([0, 0, 0,\
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

def ADRC():
    pass

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