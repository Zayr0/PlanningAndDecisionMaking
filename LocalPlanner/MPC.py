import pybullet as p
#from Environment.world import build_world
from Environment.Environment import Environment
from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import *
import time
import numpy as np
from GlobalPlanner.RRT import RRT
import matplotlib.pyplot as plt
import cvxpy as cp
from GlobalPlanner.SafeFlightPolytope import *
from Helper.HullMaths import *

def mpc(drone, x0, x_goal, A_ineq, b_ineq, Q=np.eye(12), R=np.eye(4), N=100, render=True, deltaB=None):
    x = cp.Variable((12, N))
    u = cp.Variable((4, N))

    obj = cp.Minimize(sum(cp.quad_form(x[:, i] - x_goal.T, Q) + cp.quad_form(u[:, i], R) for i in range(N)))

    cons = [x[:, 0] == drone.dsys.A @ x0 + drone.dsys.B @ u[:, 0]]

    for i in range(1, N):
        cons += [x[:, i] == drone.dsys.A @ x[:, i - 1] + drone.dsys.B @ u[:, i]]

        if (deltaB!=None):
            cons += [A_ineq @ x[0:3, i] <= b_ineq + np.array(deltaB).T * i]

    u_max = np.array([29.4, 1.4715, 1.4715, 0.0196])
    u_min = np.array([-9.8, -1.4715, -1.4715, -0.0196])
    cons += [u <= np.array([u_max]).T, u >= np.array([u_min]).T]

    if deltaB==None:
        cons += [A_ineq @ x[0:3, :] <= b_ineq]

    prob = cp.Problem(obj, cons)
    prob.solve(verbose=True)

    # print("optimal value", prob.value)
    # print(u.value)
    # print(x.value)

    point_id = []
    line_id = []
    if render == True:
        pred_x = np.hstack((x0[0:3].reshape((3, 1)), x.value[0:3, :]))
        for i in range(N):
            point_id.append(p.addUserDebugPoints([pred_x[:, i + 1].tolist()], [[0, 0, 1]], 5))
            line_id.append(p.addUserDebugLine(pred_x[:, i].tolist(), pred_x[:, i + 1].tolist(), [0, 1, 0], 3))

    return u.value, x.value, point_id, line_id