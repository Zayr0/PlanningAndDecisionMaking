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

### Modelling
The modelling package contains the model of the quadrotor. The dynamics and linearization procces are described here.
Also, the minimum snap generation functions are in this folder, as they are based on the drone dynamics.

### Planning
The planning package contains the global planning algorithms.
There is a class for calling RRT, RRT*, and informed RRT* all in one. 
The package also contains the safe flight polytope functions.

### Sampling (Not used)
The sampling package contains a sampling class that can sample a given space. 
There also are some usefull distance calculation and minimum distance functions.
However, these functions where integrated into the other algorithm classes, so this package became redundent.



## Work division
* finish drone dynamics python code - Leon
* connect state space to .urdf file - Leon
* sampled point in free space checking function - Jep
* Map representation -> use voxels or somethign else? - Jep
* graph generation e.g. RRT* - Long
* steering function for quadrotor (paper about snap control https://ieeexplore.ieee.org/document/5980409) - Maria
* dynamic obstacles representation - Maria
* collision avoidance - Long
* controller - Leon

* integrate min snap - Leon
* calculate closest safe point within feasible set, from which recalculation can be done
* octo tree/radius method - Jep
* Velocity Obstacles - Jep
* MPC - Long
* Metrics and Experiment
* Report
* Presentation
* Video!
