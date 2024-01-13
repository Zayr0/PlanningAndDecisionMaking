import numpy as np
from .mistgen.minimum_snap import mist_generator

def test_traj(N):
    x_x_ref = 1 * np.cos(np.arange(N)/10)
    x_y_ref = 1 * np.sin(np.arange(N)/10)
    x_z_ref = 1 * np.ones(N)
    x_ref = np.vstack((x_x_ref, x_y_ref, x_z_ref, np.zeros((9, N))))
    return x_ref


def test_traj_square(N):
    x_x_ref = 2*np.round(np.cos(np.arange(N)/100))
    x_y_ref = 2*np.round(np.sin(np.arange(N)/100))
    x_z_ref = 2 * np.ones(N)
    x_ref = np.vstack((x_x_ref, x_y_ref, x_z_ref, np.zeros((9, N))))
    return x_ref


def test_traj_wps(N, wps):
    #wps = np.array([[1, 0, 1], [1, 3, 1], [3, 3, 1]])
    n_pwc = wps.shape[0]  # number of piece wise constant elements
    n_points_per_link = int((N / n_pwc))
    n_recip = N % n_pwc
    x_x_ref = np.hstack([wps[i][0] * np.ones(n_points_per_link) for i in range(n_pwc)])
    x_y_ref = np.hstack([wps[i][1] * np.ones(n_points_per_link) for i in range(n_pwc)])
    x_z_ref = np.hstack([wps[i][2] * np.ones(n_points_per_link) for i in range(n_pwc)])
    x_xyz_ref = np.vstack((x_x_ref, x_y_ref, x_z_ref))

    x_ref = np.vstack((x_xyz_ref, np.zeros((9, N - n_recip))))
    x_ref = np.hstack((x_ref, np.repeat(x_ref[:, -1].reshape((12,1)), N-x_ref.shape[1], axis=1)))
    return x_ref


def min_snap(N, wps):
    ax, ay, az = wps[:][0], wps[:][1], wps[:][2]
    waypts_ori = np.array([ax, ay, az])

    T = int(N/100)
    v0 = np.array([0, 0, 0])
    a0 = np.array([0, 0, 0])
    ve = np.array([0, 0, 0])
    ae = np.array([0, 0, 0])

    myMistGen = mist_generator()
    xxs, yys, zzs, tts = myMistGen.mist_3d_gen(waypts_ori, v0, a0, ve, ae, T)
    vaj_xy = myMistGen.mist_3d_vaj_gen(xxs, yys, zzs, tts)
    #myMistGen.mist_3d_vis(waypts_ori, xxs, yys, zzs, tts, vaj_xy, True, True, True)
    x_ref = np.vstack([xxs, yys, zzs, np.zeros((9,xxs.shape[0]))])
    return x_ref
