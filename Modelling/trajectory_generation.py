import numpy as np


def test_traj(N):
    x_x_ref = 1 * np.cos(np.arange(N)/10)
    x_y_ref = 1 * np.sin(np.arange(N)/10)
    x_z_ref = 1 * np.ones(N)
    x_ref = np.vstack((x_x_ref, x_y_ref, x_z_ref, np.zeros((9, N))))
    return x_ref


def test_traj_wp(N):
    x_x_ref = 2*np.round(np.cos(np.arange(N)/100))
    x_y_ref = 2*np.round(np.sin(np.arange(N)/100))
    x_z_ref = 2 * np.ones(N)
    x_ref = np.vstack((x_x_ref, x_y_ref, x_z_ref, np.zeros((9, N))))
    return x_ref