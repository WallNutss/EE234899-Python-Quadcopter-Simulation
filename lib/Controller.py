import numpy as np
import math


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

def SMC():
    pass