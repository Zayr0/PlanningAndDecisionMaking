import numpy as np
import matplotlib.pyplot as plt

from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import test_traj


drone = Quadrotor()
N = 100
x_bag, u_bag = drone.get_ss_bag_vectors(N)  # arrays to bag the historical data of the states and inputs

x_ref = test_traj(N)

x0 = np.array([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
u0 = np.array([0, 0, 0, 0])
x_bag[:, 0] = x0
u_bag[:, 0] = u0

for k in range(N-1):
    x_bag[:, k+1] = drone.step(x_bag[:, k], x_ref[:, k])

plt.plot(x_bag[0, :], '#1f77b4', label="x_x")
plt.plot(x_ref[0,:], '#1f77b4', linestyle='--', label="x_x ref")
plt.plot(x_bag[1, :], '#ff7f0e', label="x_y")
plt.plot(x_ref[1,:], '#ff7f0e', linestyle='--', label="x_y ref")
plt.plot(x_bag[2, :], '#2ca02c', label="x_z")
plt.plot(x_ref[2,:], '#2ca02c', linestyle='--', label="x_z ref")
plt.legend()
plt.show()

print("----- A continuous -----")
print(np.round(drone.csys.A, 2))
print("------ A discrete ------")
print(np.round(drone.dsys.A, 2))
print("------ LQR gain K ------")
print(np.round(drone.K, 2))
print("------ B discrete ------")
print(np.round(drone.dsys.B, 2))
