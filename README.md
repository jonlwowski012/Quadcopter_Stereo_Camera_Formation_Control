# Quadcopter Stereo Camera Formation Control
This is a simulation of a Quadcopter Stereo Camera Formation Control that uses the nonlinear quadcopter simulator found at https://github.com/abhijitmajumdar/Quadcopter_simulator

![Quadcopter Stereo Camera Formation Control](/formctrl.gif?raw=true "formctrl")

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

## How to Download 
Clone the repository (into catkin_ws if using ROS)
```sh
$ git clone https://github.com/jonlwowski012/Quadcopter_Stereo_Camera_Formation_Control.git
```
## How to run Linear Simulation
Clone the repository, move into the directory, and run the code:
```sh
$ cd Quadcopter_Stereo_Camera_Formation_Control/Linear
```
State feedback formation control simulator:
```sh
$ python uav_model_sfb.py
```
PID formation control simulator:
```sh
$ python uav_model_pid.py
```
Model Predictive Control formation control simulator (Pick one below):
```sh
$ python uav_model_small_window_mpc.py
$ python uav_model_medium_window_mpc.py
$ python uav_model_large_window_mpc.py
```
Lyapunov Bang Bang (Not traditional bang bang) formation control simulator:
```sh
$ python uav_model_bang.py


## How to run Nonlinear Simulation
State feedback formation control simulator:
```sh
$ python multiquad_sim_SFB.py
```
PID formation control simulator:
```sh
$ python multiquad_sim_PID.py
```
Model Predictive Control formation control simulator (Pick one below):
```sh
$ python multiquad_sim_small_window_mpc.py
$ python multiquad_sim_medium_window_mpc.py
$ python multiquad_sim_large_window_mpc.py
```
Lyapunov Bang Bang (Not traditional bang bang) formation control simulator:
```sh
$ python multiquad_sim_bangbang.py
```

## How to run Gazebo/ROS Simulation
Download and Install ROS Kinetic (http://wiki.ros.org/kinetic/Installation)

State feedback formation control simulator:
```sh
$ roslaunch GazeboSimulation SFB_Controller.launch 
```
PID formation control simulator:
```sh
$ roslaunch GazeboSimulation PID_Controller.launch 
```
Model Predictive Control formation control simulator (Pick one below):
```sh
$ roslaunch GazeboSimulation MPC_Small_Window_Controller.launch 
$ roslaunch GazeboSimulation MPC_Medium_Window_Controller.launch 
$ roslaunch GazeboSimulation MPC_Large_Window_Controller.launch 
```
Lyapunov Bang Bang (Not traditional bang bang) formation control simulator:
```sh
$ roslaunch GazeboSimulation BANG_Controller.launch 

Start the UAV Motors
```sh
$ rosservice call /uav1/enable_motors start
$ rosservice call /uav2/enable_motors start
```


```
