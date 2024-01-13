import pybullet as p
#from Environment.world import build_world
from Environment.Environment import Environment
from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import *
import time
import numpy as np
from Planning.RRT import RRT
import matplotlib.pyplot as plt
import cvxpy as cp
from Planning.SafeFlightPolytope import *
from Helper.HullMaths import *
from MPC import mpc


# connect
p.connect(p.GUI)
p.setGravity(0, 0, -10)
N = 300  # number of simulation steps

# build the Environment by loading obstacle .urdfs and obtaining their IDs
staticBounds = Bounds([[-5, 5], [-5, 5], [-5, 5]], center=[0, -6, 5])
staticEnv = Environment(type="Static", bounds=staticBounds)
start = [0, -10, 5]


# load the drone and specify its dynamics
# droneID = p.loadURDF("sphere2.urdf", start, p.getQuaternionFromEuler([0, 0, 0]))
drone = Quadrotor()
prox_radius = 10.0
A_ineq, b_ineq, vertices = get_sfp(np.array(start), staticEnv, polytope_vertices=True, proximity_radius=prox_radius)
sfp_id = draw_polytope2(vertices)
goal = [1, -9, 6]
start = [0, -10, 5]
# p.removeUserDebugItem(droneID)

p.addUserDebugPoints([goal], [[0, 0, 1]], 5)
p.addUserDebugPoints([start], [[0, 0, 1]], 5)

x0 = np.array(start + [0]*9)
x_goal = np.array(goal + [0]*9)
Q = np.eye(12)
R = np.eye(4)
N = 100

u, x, point_id, line_id = \
    mpc(drone, x0, x_goal, A_ineq, b_ineq, Q=np.eye(12), R=np.eye(4), N=100, render=True)

x = cp.Variable((12, N))
u = cp.Variable((4, N))

obj = cp.Minimize(sum(cp.quad_form(x[:,i]-x_goal.T, Q)+cp.quad_form(u[:,i], R) for i in range(N)))

cons = [x[:, 0] == drone.dsys.A@x0 + drone.dsys.B@u[:, 0]]

for i in range(1, N):
    cons += [x[:, i] == drone.dsys.A@x[:, i-1] + drone.dsys.B@u[:, i]]

u_max = np.array([29.4, 1.4715, 1.4715, 0.0196])
u_min = np.array([-9.8, -1.4715, -1.4715, -0.0196])
cons += [u <= np.array([u_max]).T, u >= np.array([u_min]).T]
cons += [A_ineq@x[0:3, :] <= b_ineq]

prob = cp.Problem(obj, cons)
prob.solve(verbose=True)

# print("optimal value", prob.value)
# print(u.value)
# print(x.value)

pred_x = np.hstack((np.array([start]).T, x.value[0:3, :]))
for i in range(N):
    p.addUserDebugPoints([pred_x[:,i+1].tolist()], [[0, 0, 1]], 5)
    p.addUserDebugLine(pred_x[:,i].tolist(), pred_x[:,i+1].tolist(), [0, 1, 0], 3)

# p.removeUserDebugItem(droneID)

a=1