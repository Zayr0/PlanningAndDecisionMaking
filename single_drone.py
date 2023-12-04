"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ python pid.py

Notes
-----
The drones move, at different altitudes, along cicular trajectories
in the X-Y plane, around point (0, -.3).

"""
import os
import time
import numpy as np
import pybullet as p
import pybullet_data
#shit gym_pybullet_drones packages
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl


"""create the environment"""
p.connect(p.GUI)
p.resetSimulation()
# load the plane
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

# load the obstacles
p.setAdditionalSearchPath(os.path.join(os.getcwd(), "assets"))
p.loadURDF("leonshouse.urdf", [-.5, -.5, .05], p.getQuaternionFromEuler([0, 0, 0]))

# load the quadrotor
droneID = p.loadURDF('cf2x.urdf', [1, 1, 1], p.getQuaternionFromEuler([0, 0, 0]))

"""control stuff"""

""""follow a trajectory"""
n_steps = 30
def trajectory_circle(step, r=1):
    c_theta = 2*np.pi/n_steps #scaling factor to scale to full circle
    return [r*np.cos(c_theta*step), r*np.sin(c_theta*step), 0.1*step]

"""sample obstacles"""

""""run simulation"""
ctrl = DSLPIDControl(drone_model=DroneModel("cf2x"))
action = np.zeros((1,4))
for step in range(n_steps):
    p.stepSimulation()
    time.sleep(0.3)