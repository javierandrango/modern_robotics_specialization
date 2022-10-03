# Modern Robotics Course 4 (Robot Motion Planning and Control) A* Search

## Introduction

implementation of A* search. 
Given a graph, the start node, and the goal node, the program searches the graph for a minimum-cost path from the start to the goal.  
The program either returns a sequence of nodes for a minimum-cost path (ordered from low to high value) or indicates that no solution exists.

## Results

### Simulation (edges.csv, nodes.csv, obstacles.csv, path.csv): 
<p align="center">
  <img width="35%" height="35%" src="/Course_4_Robot_Motion_Planning_and_Control/images/a_star_search.gif">
</p>

## Usage

-results folder:
  - edges.csv : readable file in `main.py`
  - nodes.csv : readable file in `main.py`
  - obstacles.csv : file for simulation only
  - path.csv : generated file in `main.py`    

1. The relevant functions and the A* search algorithm as a function are described in the file:
```
Astar.py
```

2. The main code is described in the file:
```
main.py
```

3. Open the file `main.py` in an editor and run the code.

**NOTE**: The program automatically generates the `path.csv` file in the folder `results`

4. Open CoppeliaSim with the scene `Scene5_motion_planning.ttt` provided in the course
5. Copy the path of the folder `results` that contains the csv files in the scene and run thr program

## Mainteiners

The content described belongs to the owner of this repository and was developed for educational purposes only. 
