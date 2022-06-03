# Modern Robotics Course 3 (Robot Dynamics) Project

## Introduction

Code that simulates the motion of the UR5 for a specified amount of time (in seconds), from a specified initial configuration (at zero velocity), when zero torques are applied to the joints.  In other words, the robot simply falls in gravity. Gravity is $g=9.81 m/s^2$ in the $−\hat{z}$ direction, i.e., gravity acts downward. The motion should be simulated with at least $100$ integration steps per second. The program calculate and record the robot joint angles at each step. This data were saved as a .csv file, where each row has six numbers separated by commas. This .csv file is suitable for animation with the CoppeliaSim UR5 csv animation scene.

## Usage

1. The relevant kinematic and inertial parameters of the UR5 (6-DOF industrial robot arm) were described in the file:
```
UR5_parameter.py
```

2. The main code is described in the file:
```
main.py
```
    Open the file in an editor and run the code.

**NOTE**:


## Screenshots


## Mainteiners

The content described belongs to the owner of this repository and was developed for educational purposes only. 

