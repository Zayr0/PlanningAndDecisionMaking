import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def local_planner():
    df = pd.read_csv('local_planner_dynamic_MPC.csv', header=0)
    mpc = np.array(df)
    mpc_execution_time = [mpc[k, 0] for k in range(mpc.shape[0]) if mpc[k, 1] == 1]
    df = pd.read_csv('local_planner_dynamic_LQR.csv', header=0)
    lqr = np.array(df)
    print(lqr.shape)
    lqr_execution_time = [lqr[k, 0]*10 for k in range(lqr.shape[0]) if mpc[k, 1] == 1]
    plt.hist([mpc_execution_time, lqr_execution_time], label=["MPC + SFP", "Stop and go"])
    plt.xlabel("time [s]")
    plt.ylabel("count")
    plt.title("Execution time local planner dynamic env")
    plt.legend()
    plt.show()

def global_planner():
    rrt = np.array(pd.read_csv('global_planner_rrt.csv', header=0))
    rrt_star = np.array(pd.read_csv('global_planner_rrt_star.csv', header=0))
    rrt_star_informed = np.array(pd.read_csv('global_planner_rrt_star_informed.csv', header=0))
    print(f"For \t\t\t RRT, \t RRT*, \t and informed RRT* respectively")
    print(f"Avg. solver time: {np.round(np.average(rrt[:,0]), 3)}, \t {np.round(np.average(rrt_star[:,0]), 3)}, \t {np.round(np.average(rrt_star_informed[:,0]), 3)}")
    print(f"Variance solver time: {np.round(np.var(rrt[:,0]), 3)}, \t {np.round(np.var(rrt_star[:,0]), 3)}, \t {np.round(np.var(rrt_star_informed[:,0]), 3)}")
    print(f"Avg. path length: {np.round(np.average(rrt[:, 1]), 3)}, \t {np.round(np.average(rrt_star[:, 1]), 3)}, \t {np.round(np.average(rrt_star_informed[:, 1]), 3)}")
    print(f"Variance path length: {np.round(np.var(rrt[:, 1]), 3)}, \t {np.round(np.var(rrt_star[:, 1]), 3)}, \t {np.round(np.var(rrt_star_informed[:, 1]), 3)}")


global_planner()
local_planner()
