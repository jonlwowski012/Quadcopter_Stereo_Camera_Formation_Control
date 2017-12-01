# Quadcopter simulator
A quadcopter simulator with single and multi-quad simulations. The simulator supports time scaling (including real-time simulations) and headless mode (the simulator runs in background without a GUI update).

![Single Quadcopter Simulation](/quad_sim.gif?raw=true "quad_sim")

Single Quadcopter Simulation

![Multi Quadcopter Simulation](/multiquad_sim.gif?raw=true "multiquad_sim")

Multi Quadcopter Simulation

## Dependencies
- Simulation of dynamics:
    - Numpy
    - Math
    - SciPy
- GUI:
    - Matplotlib
    - Matplotlib Mapping Toolkits
- Threading:
    - Time
    - Datetime
    - Threading

## How to run
Clone the repository, move into the directory, and run the code:
```sh
$ git clone https://github.com/abhijitmajumdar/Quadcopter_simulator.git
$ cd Quadcopter_simulator
```
Single quad simulator:
```sh
$ python quad_sim.py
```
Multi quad simulator:
```sh
$ python multiquad_sim.py
```

## Working
The main classes which define the simulator are Propeller, Quadcopter and GUI. There is also a sample Controller class, which implements a controller for the quadcopter. The objective was to make a quadcopter dynamic simulator, which allowed us to control each motor individually. The other requirement was the ability to run the simulations in the background, hence possibly expediting the computations, commonly referred to as the headless mode. Once the simulator thread is started, the GUI may or may not be updated at the developers will. There is also a time scaling factor, which can allow the simulation to run as fast as the processor supports, and as slow as one can fall asleep doing so.

##### Propeller class
This class defines the thrust generated by a propeller of a specified size at a given speed of rotation. This was based on the equation provided on http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html. This was made into a separate class to enable the implementation of other multi-rotors as well.

##### Quadcopter class
This class performs the simulations of the dynamics based on the state space solution of a quadcopter. It uses 4 objects of the Propeller class to implement the quad configuration of a quadcopter. The state space representation of a quadcopter model have been adapted from Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky and Quadrotor Dynamics and Control by Randal Beard. The class is initialized using the quadcopter parameters like length of an arm, the weight of the quadcopter, radius of a sphere representing the center blob of the quadcopter, etc. It is defined in a dictionary which can be modified.

The state space is defined as: *X = [x,y,z,x_dot,y_dot,z_dot,theta,phi,gamma,theta_dot,phi_dot, gamma_dot]*. The update to the state is performed by an ODE solver from the current state to a new state over a period of *dt* time(defined by user). It uses the *vode* ODE solver available from the *SciPy* library. It has an update method to update the state, which is run on a thread at intervals defined by the time scaling factor. The thread can be started by the *start_thread* method.

It has methods to set_motor_speeds(), get_orientation(), get_position(), get_angular_rate(), get_linear_rate(), set_position() and set_orientation(), which can be used by the controller.

##### Graphical User Interface (GUI) class
The GUI initialization takes in the same quadcopter parameter directory. It is optional to pass in the get methods for position and orientation of the quadcopter to the GUI initialization. It uses *Matplotlib* to plot a representation of the quadcopter, each update of which takes significantly more time than the quadcopter class update. It can be updated using the method update(), with the position and orientation of the quadcopter if the get methods are not defined while initializing the object.

##### Controller class
A demo controller class. It is initialized using the quadcopter object and a controller parameter dictionary. The quadcopter object is used to update the global time as well as the quadcopter state and also to set the motor speeds on the quadcopter. An example  parameter dictionary is provided which defines the different constants used by the controller. The update() method updates the motor speeds based on the control algorithm. The start_thread() method initializes the thread to keep updating the controller every *update_rate*(specified by the user).

## Parameters
- TIME_SCALING: Used to define how fast the simulator runs. Value of 1.0 corresponds to real-time simulations. Any smaller number makes the simulation faster and vice versa. A value of 0 runs the simulation run as fast as possible on the current hardware.
- QUAD_DYNAMICS_UPDATE: The delta time over which the dynamics of the quadcopter are updated
- CONTROLLER_DYNAMICS_UPDATE: The delta time over which the controller updates the motors
- QUADCOPTER: The parameters which define the quadcopter: initial position and orientation,length of arm, center radius, propeller size and weight.
- CONTROLLER_PARAMETERS: The parameters which define the controller behavior: Motor limits, Tilt limits, Throttle offset, Linear PID, Linear to Angular Scaler and Angular PID
- GOALS: The goals to loop over




## Changes
- Added gifs

## To-do
- Make separate README for single and double quadcopter simulators