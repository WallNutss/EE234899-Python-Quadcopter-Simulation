import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib import animation

# Importing class Quadcopter
from lib.quadcopter import quadcopter
# Importing rotation matrix
from lib.rotation import RPY2XYZ, D2R, R2D

# Body Frame Location for Transformation Matrix Preparation
            # Position Motor1  Motor2 Motor3  Motor4
drone_body = np.array([[-0.042, 0.042, 0.042, -0.042],  # X Position
                       [0.042 , 0.042,-0.042, -0.042],  # Y Postion
                       [    0 ,     0,     0,      0],  # Z Position
                       [     1,     1,     1,      1]]) # Adder

Quadcopter = quadcopter()

## Inisialization of plot figure

# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Rotation Matrix, when Phi=Psi==0, Theta=20, In X Configuration shape, + sign mean that the drone will move backward
angles = np.array([D2R(0), D2R(20), D2R(0)])
R = RPY2XYZ(angles).transpose()

# Adding some balancer to Transformation Matrix for position calculation update
add = np.array([0, 0, 0, 1])
# This is the transformation matrix, combine from R and init_state
wHb = np.vstack((np.hstack((R, Quadcopter.initPosition)),add))

# This is the transformation matrixs implemented on drone body
drone_world = np.dot(wHb, drone_body)

# Just taking the neccessary information
drone_altitude = drone_world[:3,:]

# Motor position so we can plot it
motor_13 = drone_altitude[:,[0,2]]
motor_24 = drone_altitude[:,[1,3]]

# Define the coordinates for the drone shape
x1 = np.array([motor_13[0,0], motor_13[0,1]])
y1 = np.array([motor_13[1,0], motor_13[1,1]])
z1 = np.array([motor_13[2,0], motor_13[2,1]])

x2 = np.array([motor_24[0,0], motor_24[0,1]])
y2 = np.array([motor_24[1,0], motor_24[1,1]])
z2 = np.array([motor_24[2,0], motor_24[2,1]])

# Connect the points to form the drone shape
motor13, = ax.plot(x1,y1,z1, '--bo', markersize=5, markerfacecolor="None", label="motor13_pos")
motor24, = ax.plot(x2,y2,z2, '--ro', markersize=5, markerfacecolor="None", label="motor24_pos")

# Set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Set plot limits
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_zlim(0, 1)

plt.grid()
plt.show()

def update_point(n):
    pass

def main():
    ani = animation.FuncAnimation(fig, update_point, interval=20, blit=True)

if __name__ == "__main__":
    main()
