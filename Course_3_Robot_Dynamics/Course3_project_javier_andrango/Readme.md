# Modern Robotics Course 3 (Robot Dynamics) Project

## Introduction

Code that simulates the motion of the UR5 for a specified amount of time (in seconds), from a specified initial configuration (at zero velocity), when zero torques are applied to the joints.  In other words, the robot simply falls in gravity. Gravity is $g=9.81m/s^2$ in the -$\hat{z}$ direction, i.e., gravity acts downward. The motion should be simulated with at least $100$ integration steps per second. The program calculate and record the robot joint angles at each step. This data were saved as a .csv file, where each row has six numbers separated by commas. This .csv file is suitable for animation with the CoppeliaSim UR5 csv animation scene.

## Results

### Simulation 1 (simulation1.csv): 
<p align="center">
  <img width="35%" height="35%" src="/Course_3_Robot_Dynamics/images/course3_project_simulation1_CoppeliaSim_scene.gif">
</p>

### Simulation 2 (simulation2.csv): 
<p align="center">
<img width="35%" height="35%" src="/Course_3_Robot_Dynamics/images/course3_project_simulation2_CoppeliaSim_scene.gif">
</p>

## Usage

1. The relevant kinematic and inertial parameters of the UR5 (6-DOF industrial robot arm) were described in the file:
```
UR5_parameter.py
```

2. The main code is described in the file:
```
main.py
```

3. Open the file `main.py` in an editor and run the code.

**NOTE**: The code takes several minutes to generate the `.csv` files after running the program



## Mainteiners

The content described belongs to the owner of this repository and was developed for educational purposes only. 

