# Quadcopter Python 3D Plot Animation
A Quadcopter simulation with 3D Plot Animation using matplotlib python package.  It contains a basic quadcopter dynamics model, hover controller, waypoint position control, and visualisation toolkit using matplotlib animation.

# Motivation
This project is my undergraduate thesis project about how to control a quadcopter using an Integrated Sliding Mode Control. This project serve me with two purpose: that is as a learning module how to build a model to explain how Quadcopter behave and as a ground level for tuning my parameters to control the Quadcopter. For me, there are already many interesting projects around already, like vision-based SLAM, hover control and advance maneuver, etc. However, there are very few open-source quadcopter simulator that helps a beginner(possibly me) to overcome the mental barrier of understanding the underlying physics behind the model of the Quadcopter and a simple plot animation for visualizing this simulation for easy debugging. This project is a simple Quadcopter simulation in 3D Plot Animation, where I was heavily inspired by this code structure from [SKYnSPACE](https://github.com/SKYnSPACE/AE450/tree/master/Lec10). I hope this helps for anyone to learn how Quadcopter works

# How to Run


# System Control Structure


# What you can change


# Known Issues

# Possible Development


# Useful Links


with cascaded controller system where where the position error (difference between the desired position and the current position) generates a desired angle setpoint, then from there we have an attitude controller which creates a desired thrust magnitude and orientation,based on those values. 
