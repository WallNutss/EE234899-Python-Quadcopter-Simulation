# Quadcopter Python 3D Plot Animation
A Quadcopter simulation with 3D Plot Animation using matplotlib python package.  It contains a basic quadcopter dynamics model, hover controller, waypoint position control, and visualisation toolkit using matplotlib animation.

# Motivation
This project is my undergraduate thesis project about how to control a quadcopter using an Integrated Sliding Mode Control. This project serve me with two purpose: that is as a learning module how to build a model to explain how Quadcopter behave and as a ground level for tuning my parameters to control the Quadcopter. For me, there are already many interesting projects around already, like vision-based SLAM, hover control and advance maneuver, etc. However, there are very few open-source quadcopter simulator that helps a beginner(possibly me) to overcome the mental barrier of understanding the underlying physics behind the model of the Quadcopter and a simple plot animation for visualizing this simulation for easy debugging. This project is a simple Quadcopter simulation in 3D Plot Animation, where I was heavily inspired by this code structure from [SKYnSPACE](https://github.com/SKYnSPACE/AE450/tree/master/Lec10). I hope this helps for anyone to learn how Quadcopter works

# How to Run
Clone the repository, move into the directory, and run the code:
```sh
$ git clone https://github.com/WallNutss/EE234899-Python-Quadcopter-Simulation.git
$ cd EE234899-Python-Quadcopter-Simulation
$ python run3DSimulation.py
```

# What you can change

    ├── run3DSimulation.py     # main file to run, it contains the Quadcopter Class representation for dynamics update
    ├── lib                    # support files where it contains the controller, rotation matrix, quadcopter parameters class
    ├── windModel              # Code for Wind Model based on Dryden Wind Turbulence Model
    └── README.md
    
In this simulation, I use a common cascade controller which is the first one is the position controller using a PID Controller and the second one is an attitude controller where I have two types of this controller. Which is using a PID Controller and five modifications of SMC Controller, namely SMC-Sign, SMC-Saturation, SMC-Tanh, and Integral SMC-Tanh. The files and folder structure listen above. If you find any error or bug please let me know.

So it the `run3DSimulation.py`, it's the main file you want to run, where you can find the class variable Quadcopter. Here I have provided a simple change parameter you can use
* Ts(Time Sampling) : Used to define how fast the simulation runs. Value of in format of seconds. Default to 0.02s. Used to define how fast the simulator runs.
* controller : a decision variable, use 0 for choosing PID as an attitude controller and 1 for SMC Controller
* smctype : a decision variable, there are 5 mode for this, feel free to choose them, details in file `lib\controller` SMC function
* DISTURBANCE : Set `True` to apply wind disturbance to simulation

in `lib`, there is little you can change, but hey feel free to break it. Where in `windModel` it contains the Dryden Wind Turbulence Model. You can change the possible height of the Quadcopter is flying for better simulation prediction.

# System Control Structure


# Known Issues


# Possible Development


# Useful Links


with cascaded controller system where where the position error (difference between the desired position and the current position) generates a desired angle setpoint, then from there we have an attitude controller which creates a desired thrust magnitude and orientation,based on those values. 
