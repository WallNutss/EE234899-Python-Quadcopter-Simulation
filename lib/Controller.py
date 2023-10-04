import numpy as np
import math

def sat(v):
    v = np.copy(v)
    v[np.abs(v) > 0.5] = np.sign(v[np.abs(v) > 0.5])
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

def SMC(sliding, inert, anglevelocity, lamda, prevAngle):
    K = np.array([0.29,      0,    0, \
                    0,    0.29,    0, \
                    0,      0,  0.01]).reshape(3,3)
    # Let him cook
    inert_inv = np.linalg.inv(inert)

    # Part A, the Control Equation for attitude
    Iw   = np.dot(inert, anglevelocity) # For the I.[p,q,r]
    Invw = np.dot(inert_inv, anglevelocity) # For the I^-1.[p,q,r]

    Inertia_F = np.cross(Invw[:,0], Iw[:,0]).reshape(3,1)
    U_eq = inert.dot(Inertia_F - inert_inv.dot(0.004609 * anglevelocity) - lamda.dot(anglevelocity))
    #U_eq = inert.dot(Inertia_F - lamda.dot(anglevelocity))

    sliding[0] = sat(sliding[0])
    sliding[1] = sat(sliding[0])
    sliding[2] = sat(sliding[0])

    U_SMC = U_eq - K.dot(sliding)

    print(U_SMC)
    print("-------------------------")
    return U_SMC

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