import numpy as np
import math 
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)

# Importing class Quadcopter
from lib.quadcopter import quadcopter
# Importing rotation matrix
from lib.rotation import RPY2XYZ, D2R, R2D
# Import Anti Windup
from lib.controller import antiWindup

# Body Frame Location for Transformation Matrix Preparation
            # Position Motor1  Motor2 Motor3  Motor4, MoC , LoS
drone_body = np.array([[-0.042, 0.042, 0.042, -0.042, 0.0,   0.0],  # X Position
                       [0.042 , 0.042,-0.042, -0.042, 0.0, 0.042],  # Y Postion
                       [    0 ,     0,     0,      0, 0.0,   0.0],  # Z Position
                       [     1,     1,     1,      1,   1,     1]]) # Adder


Quadcopter = quadcopter(Ts=0.02)

# ------------- Inisialization of plot figure ------------- #
# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
state_display = ax.text2D(0.17, 0.90, "green" ,color='green', transform=ax.transAxes)
time_display = ax.text2D(0.17, 0.85, "red" ,color='red', transform=ax.transAxes)

# Create a another figure for angle
fig2, ax2 = plt.subplots(3,1)
xLogs, = ax2[0].plot([],[],'g', lw=2)
ax2[0].set_xlim(0, 10)
ax2[0].set_ylim(-5, 5)
ax2[0].set_xlabel('X [m]')
ax2[0].set_ylabel('Y [m]')

yLogs, = ax2[1].plot([],[], 'r', lw=2)
ax2[1].set_xlim(0, 10)
ax2[1].set_ylim(-5, 5)
ax2[1].set_xlabel('X [m]')
ax2[1].set_ylabel('Y [m]')

zLogs, = ax2[2].plot([],[], 'm',lw=2)
ax2[2].set_xlim(0, 10)
ax2[2].set_ylim(0, 5)
ax2[2].set_xlabel('X [m]')
ax2[2].set_ylabel('Y [m]')

# Rotation Matrix, when Phi=Psi==0, Theta=20, In X Configuration shape, + sign mean that the drone will move backward, event though I dont know which forward or backward when in simulation. If academia cares about my undergraduate thesis, they should pay attention more to the student who is strugling, which in case they didn't yeah so fuck it its really long comment btw, wanna hang out?
angles = np.array([D2R(0), D2R(0), D2R(0)])
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
lineofsight = drone_altitude[:, [4,5]]

# Define the coordinates for the drone shape
x1 = np.array([motor_13[0,0], motor_13[0,1]])
y1 = np.array([motor_13[1,0], motor_13[1,1]])
z1 = np.array([motor_13[2,0], motor_13[2,1]])

x2 = np.array([motor_24[0,0], motor_24[0,1]])
y2 = np.array([motor_24[1,0], motor_24[1,1]])
z2 = np.array([motor_24[2,0], motor_24[2,1]])

x3 = np.array([lineofsight[0,0], lineofsight[0,1]])
y3 = np.array([lineofsight[1,0], lineofsight[1,1]])
z3 = np.array([lineofsight[2,0], lineofsight[2,1]])

# Connect the points to form the drone shape
motor13, = ax.plot(x1,y1,z1, '--bo', markersize=3, markerfacecolor="None", label="motor13_pos")
motor24, = ax.plot(x2,y2,z2, '--ro', markersize=3, markerfacecolor="None", label="motor24_pos")
los,     = ax.plot(x3,y3,z3, '--k', markersize=3, markerfacecolor="None", label="lineofsight")

# Set axis labels
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')

ax.xaxis.set_major_locator(MultipleLocator(5))
ax.yaxis.set_major_locator(MultipleLocator(5))
ax.zaxis.set_major_locator(MultipleLocator(1))

# Set plot limits
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_zlim(0, 5)

phi_ref     = D2R(0)
theta_ref   = D2R(0)
psi_ref     = D2R(0)

AngleRef = np.array([phi_ref, theta_ref, psi_ref])

x_des       = 4.2
y_des       = 5.1
z_des       = 3.3

PositionRef = np.array([x_des, y_des, z_des])
# Random Position
#PositionRef = np.array([np.random.uniform(-5,5), np.random.uniform(-5,5),np.random.uniform(0.3,0.5)])

# ax.plot(np.array([0.0, PositionRef[0]]),np.array([0.0, PositionRef[1]]),np.array([0.0, PositionRef[2]]), '--r')
ax.plot(PositionRef[0],PositionRef[1],PositionRef[2], 'ro', markersize='3')
# ax2[0].plot(Quadcopter.timeLogs[:], Quadcopter.angleLogs[:,0])

# ------------- Start Simulation ------------- #
def update_point(n):
    """
    Perform simulation based on matplotlib function animation
    Args
        n
    Return
        Line of the figure contain position
    """
    
    # Calculate dynmamic again for model precision so we can get the newest state from the drone
    Quadcopter.DynamicSolver()
    # Planner for Yaw Reference
    Quadcopter.psi_des = antiWindup(math.atan2( Quadcopter.state[1] - PositionRef[1], Quadcopter.state[0]- PositionRef[0]), -0.99, 0.99)
    # Quadcopter for Position Controller, Where Outer Loop will
    Quadcopter.positionController(PositionRef)

    innerLoop = 4
    for i in range(0, innerLoop): 
        # Control the unit of the quadcopter
        Quadcopter.attitudeController()
        # Updating the state, in here there lies the calculation of the dynamics model and integration from the result to form original state
        Quadcopter.updateState()

        states = Quadcopter.fetchUpdate()

        # Form the result of the position we got from calculation to Inertial Frame / the 3D plot
        R = RPY2XYZ(states[6:9]).transpose()
        # This is the transformation matrix, combine from R and init_state
        wHb = np.vstack((np.hstack((R, states[0:3])),add))

        # This is the transformation matrixs implemented on drone body
        drone_world = np.dot(wHb, drone_body)

        # Just taking the neccessary information
        drone_altitude = drone_world[:3,:]
        # Motor position so we can plot it
        motor_13 = drone_altitude[:,[0,2]]
        motor_24 = drone_altitude[:,[1,3]]
        lineofsight = drone_altitude[:, [4,5]]

        # Define the coordinates for the drone shape
        x1 = np.array([motor_13[0,0], motor_13[0,1]])
        y1 = np.array([motor_13[1,0], motor_13[1,1]])
        z1 = np.array([motor_13[2,0], motor_13[2,1]])

        x2 = np.array([motor_24[0,0], motor_24[0,1]])
        y2 = np.array([motor_24[1,0], motor_24[1,1]])
        z2 = np.array([motor_24[2,0], motor_24[2,1]])

        x3 = np.array([lineofsight[0,0], lineofsight[0,1]])
        y3 = np.array([lineofsight[1,0], lineofsight[1,1]])
        z3 = np.array([lineofsight[2,0], lineofsight[2,1]])

        motor13.set_data(x1,y1)
        motor24.set_data(x2,y2)
        los.set_data(x3,y3)

        motor13.set_3d_properties(z1)
        motor24.set_3d_properties(z2)
        los.set_3d_properties(z3)

        # Add some info about current position in plot
        time_display.set_text('Simulation time = %.2fs' % (Quadcopter.Time))
        state_display.set_text('Position of the quad: \n x = %.1fm y = %.1fm z = %.1fm' % (states[0], states[1], states[2]))

        #Comment this line if you don't require the trail that is left behind the quadrotor
        #ax.scatter(states[0], states[1], states[2], "g.")
        #Logs Angle Data
        #print(Quadcopter.posLogs[:,0])
        xLogs.set_data(Quadcopter.timeLogs, Quadcopter.posLogs[:,0].flatten())
        yLogs.set_data(Quadcopter.timeLogs, Quadcopter.posLogs[:,1])
        zLogs.set_data(Quadcopter.timeLogs, Quadcopter.posLogs[:,2])


    return motor13, motor24, los, state_display, time_display, xLogs, yLogs, zLogs
  
ani = animation.FuncAnimation(fig, update_point, interval=50, blit=True)
#plt.grid()
plt.show()