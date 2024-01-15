# PlanningAndDecisionMaking

## installation instructions
* Clone the repo `git clone https://github.com/Zayr0/PlanningAndDecisionMaking.git` or extract the zipfile with code.
* Run `conda env create -f pdm_environment.yml`


## Contents
The main executable python file is the `simulation.py`. 
Running this file will show the drone using informed RRT* to generate a path through a static randomized environment.
Then a minimum snap trajectory will be generated to follow this path. 
Using safe flight polytopes and MPC the drone the flies through the environment whilst planning locally.
Then the drone uses safe flight polytopes and MPC again to move through a now dynamic environment with moving obstacles.

### CollisionDetecttion (Not used)
Contains a couple classes and test script for a LIDAR imitation, and collision detections using pybullet's collision mask API.

### Environment
The Environment package contains a class for creating environments. 
These can be set to be either "Dynamic", "Dynamic2D", and "Static. 
This package also contains classes for static randomized polytope obstacles and dynamic MovingSpheres obstacles.

### Evaluation
The evaluation package contains classes and fucntions for capturing the performance of the different algorithms used.

### Helper
The helper package contains helper functions and classes such as, bounding boxes, distance functions, rotation matrices and functions for generating convex hulls from points.

### LocalPlanner
The local planner package contains a function that can be called to implement an MPC controller.
the `test_local_planner.py` file also contains an implementation of this function.

The local planner package also contains a velocity obstacle package and class, that was never finished (95% done).
It uses linear approximations of the VO exclusion area and sampling to generate a new velocity based on the desired velocity.
There is an issue with the samples made not being rejected in the are of VO for each obstacle, depending on the position of the drone and the position of the obstacle.

### Modelling
The modelling package contains the model of the quadrotor. The dynamics and linearization procces are described here.
Also, the minimum snap generation functions are in this folder, as they are based on the drone dynamics.

### GlobalPlanner
The planning package contains the global planning algorithms.
There is a class for calling RRT, RRT*, and informed RRT* all in one. 
The package also contains the safe flight polytope functions.

### Sampling (Not used)
The sampling package contains a sampling class that can sample a given space. 
There also are some usefull distance calculation and minimum distance functions.
However, these functions where integrated into the other algorithm classes, so this package became redundent.


## Documentation/Report

## Gallery

## References
