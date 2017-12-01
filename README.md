# Quadcopter Stereo Camera Formation Control
This is a simulation of a Quadcopter Stereo Camera Formation Control that uses the nonlinear quadcopter simulator found at https://github.com/abhijitmajumdar/Quadcopter_simulator

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
$ git clone https://github.com/jonlwowski012/Quadcopter_Stereo_Camera_Formation_Control.git
$ cd Quadcopter_Stereo_Camera_Formation_Control
```
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
$ multiquad_sim_small_window_mpc.py
$ multiquad_sim_medium_window_mpc.py
$ multiquad_sim_large_window_mpc.py
```
Lyapunov Bang Bang (Not traditional bang bang) formation control simulator:
```sh
$ python multiquad_sim_bangbang.py
```
