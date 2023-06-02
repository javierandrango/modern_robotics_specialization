# Modern Robotics Specialization

## Introduction

Robotics Specialization using modern concepts in robotics is available as MOOC(massive open online course) at [Coursera](https://www.coursera.org/specializations/modernrobotics)
. A study of kinematics, Dynamics, motion planning, and control of mobile robots and robot arms. The specialization is divided into 5 courses and a Capstone project:
1. Foundation of Robot Motion
2. Robot Kinematics
3. Robot Dynamics
4. Robot motion planning and control
5. Robot manipulation and wheeled mobile robots
6. Capstone project: mobile manipulator

## Results

- -The results of every exercise represent one solution of many possibilities and were made from scratch following basic instructions within every course and researching information inside forums (stack overflow). 

- The results were saved in Jupyter Notebooks and Python files.
  

## Usage

- The original Github repository can be found [here](https://github.com/NxRLab/ModernRobotics), the repository includes step-by-step instructions to follow; also the functions are available for Matlab and Matematica.

- The Python code (ModernRobotics library) is commented and mostly self-explanatory in conjunction with the book 
[Modern Robotics: Mechanics, Planning, and Control," Kevin M. Lynch and Frank C. Park](http://hades.mech.northwestern.edu/index.php/Modern_Robotics). 
An example use is provided with each function.

  To use with Python open a command windows and install the package:
  ```cmd
  pip install modern-robotics
  ```
  Open a New Python File and import the library
  ```python
  import modern_robotics as mr
  import numpy as np
  ```
 - For simulation exercises is necessary to install the robotics simulator CoppeliaSim
    - I used a laptop running Windows 11, 6Gb RAM and the simulation run well all the time.
    - Install [CoppeliaSim](https://www.coppeliarobotics.com/downloads) direct from the official page.
    - The scenes for the exercises can be found [here](http://hades.mech.northwestern.edu/index.php/CoppeliaSim_Introduction)
    - Unzip the downloaded file into any directory of your preference. The scenes for simulation have `.ttt` extension. 
    - To run the simulations open `CopeliaSim` and import one of the scenes and follow the instructions to upload the `.scv` file (The simulations are part of the programming challenge in every course). 

## Mainteiners

The content described belongs to the owner of this repository and was developed for educational purposes only. My purpose in making this repository is just to practice and gain experience in Python, write repositories into Github, and study Robotics.  

