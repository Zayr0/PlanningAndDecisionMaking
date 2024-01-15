# PlanningAndDecisionMaking

## installation instructions
* Clone the repo `git clone https://github.com/Zayr0/PlanningAndDecisionMaking.git` or extract the zip file with code.
* Run `conda env create -f pdm_environment.yml`


## Contents
The main executable python file is the `simulation.py`. 
Running this file will show the drone using informed RRT* to generate a path through a static randomized environment.
Then a minimum snap trajectory will be generated to follow this path. 
Using safe flight polytopes and MPC the drone flies through the environment whilst planning locally.
Then the drone uses safe flight polytopes and MPC again to move through a now dynamic environment with moving obstacles.

### CollisionDetecttion (Not used)
Contains a couple of classes and a test script for a LIDAR imitation, and collision detections using Pybullet's collision mask API.

### Environment
The Environment package contains a class for creating environments. 
These can be set to be either "Dynamic", "Dynamic2D", or "Static. 
This package also contains classes for static randomized polytope obstacles and dynamic MovingSpheres obstacles.

### Evaluation
The evaluation package contains classes and functions for capturing the performance of the different algorithms used.

### Helper
The helper package contains helper functions and classes such as bounding boxes, distance functions, rotation matrices, and functions for generating convex hulls from points.

### LocalPlanner
The local planner package contains a function that can be called to implement an MPC controller.
the `test_local_planner.py` file also contains an implementation of this function.

The local planner package also contains a velocity obstacle package and class, that was never finished (95% done).
It uses linear approximations of the VO exclusion area and sampling to generate a new velocity based on the desired velocity.
There is an issue with the samples made not being rejected in the area of VO for each obstacle, depending on the position of the drone and the position of the obstacle.

### Modelling
The modeling package contains the model of the quadrotor. The dynamics and linearization processes are described here.
Also, the minimum snap generation functions are in this folder, as they are based on the drone dynamics.

### GlobalPlanner
The planning package contains the global planning algorithms.
There is a class for calling RRT, RRT*, and informed RRT* all in one. 
The package also contains the safe flight polytope functions.

### Sampling (Not used)
The sampling package contains a sampling class that can sample a given space. 
There also are some useful distance calculations and minimum distance functions.
However, these functions were integrated into the other algorithm classes, so this package became redundant.

## Documentation/Report

## Gallery
MPC through dynamic obstacles with a horizon N=20:
https://github.com/Zayr0/PlanningAndDecisionMaking/assets/50660564/5f0d034c-4349-4536-945b-bd8f991bbbe6

RRT* generating a graph. Minimum snap generates reference trajectory and MPC tracks reference trajectory.
https://github.com/Zayr0/PlanningAndDecisionMaking/assets/50660564/01d4e956-7e1a-46d7-ad37-9507694b9816

Informed RRT generating a graph. Minimum snap generates reference trajectory and MPC tracks reference trajectory.
https://github.com/Zayr0/PlanningAndDecisionMaking/assets/50660564/3b1ae8b4-56e6-4bc6-ade9-77affd880ca0

## References
![image](https://github.com/Zayr0/PlanningAndDecisionMaking/assets/50660564/40123d73-e824-47e6-a00a-f22d5620f7f1)

