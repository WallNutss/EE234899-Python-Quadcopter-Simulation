import numpy as np
import math

# Importing class Quadcopter
from lib.quadcopter import quadcopter

# Body Frame Location for Transformation Matrix Preparation
            # Position Motor1  Motor2 Motor3  Motor4
drone_body = np.array([[-0.042, 0.042, 0.042, -0.042],\
                       [0.042 , 0.042,-0.042, -0.042],\
                       [    0 ,     0,     0,     0],\
                       [     1,     1,     1,     1]])

Quadcopter = quadcopter()
print(Quadcopter)
