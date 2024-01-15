"""here performance metrics are compares"""
from simulation import run
import pandas as pd
import numpy as np


def global_plan(n_trials=3, planner="rrt"):
    experiment_name = "global_planner_" + str(planner)
    n_trials = n_trials
    settings = {}
    settings["planner"] = planner
    settings["env_static"] = True
    settings["env_dynamic"] = False
    settings["dynamic_controller"] ="MPC"
    for i in range(n_trials):
        print(planner, ": ", i)
        results = {}
        try:
            results = run(auto_testing=True, settings=settings)
        except:
            continue
        columns = ['Solver Time', 'Path Length']
        filename = "Results/" + experiment_name + ".csv"
        try:
            df = pd.read_csv(filename, header=0, usecols=columns)
        except:
            df = pd.DataFrame(columns=columns)
        results_keep = [results["result_gp_solver_time"], results["result_gp_path_length"]]
        print(results_keep)
        df.loc[len(df)] = results_keep
        # df = df.concat(dict(zip(df.columns, metrics)), ignore_index=True)
        df.to_csv(filename, index=False)
    return


def local_planner_dynamic_MPC(n_trials=3):
    experiment_name = "local_planner_dynamic_MPC"
    settings = {"env_static": False, "env_dynamic": True, "dynamic_controller": "MPC"}
    results = {}
    for _ in range(n_trials):
        try:
            results = run(auto_testing=True, settings=settings)
            #results["rslt_dynamic_success"] = 1
        except:
            results["rslt_dynamic_success"] = 0
        columns = ['Execution Time', 'Success']
        filename = "Results/" + experiment_name + ".csv"
        try:
            df = pd.read_csv(filename, header=0, usecols=columns)
        except:
            df = pd.DataFrame(columns=columns)
        results_keep = [results["rslt_dynamic_time"], results["rslt_dynamic_success"]]
        df.loc[len(df)] = results_keep
        # df = df.concat(dict(zip(df.columns, metrics)), ignore_index=True)
        df.to_csv(filename, index=False)
    return

def local_planner_dynamic_LQR(n_trials=3):
    experiment_name = "local_planner_dynamic_LQR"
    # settings = {}
    # settings["planner"] = "rrt"
    # settings["env_static"] = False
    # settings["env_dynamic"] = False
    # settings["dynamic_controller"] = "MPC"
    # settings = {"env_static": False, "env_dynamic": True, "dynamic_controller": "LQR"}
    # results = {}
    for _ in range(n_trials):
        # try:
        #     results = run(auto_testing=True, settings=settings)
        #     #results["rslt_dynamic_success"] = 1
        # except:
        #     continue
        columns = ['Execution Time', 'Success']
        filename = "Results/" + experiment_name + ".csv"
        try:
            df = pd.read_csv(filename, header=0, usecols=columns)
        except:
            df = pd.DataFrame(columns=columns)
        results_keep = [results["rslt_dynamic_time"], results["rslt_dynamic_success"]]
        print(results_keep)
        df.loc[len(df)] = results_keep
        # df = df.concat(dict(zip(df.columns, metrics)), ignore_index=True)
        df.to_csv(filename, index=False)
    return

#local_planner_dynamic_MPC(n_trials=100)
local_planner_dynamic_LQR(n_trials=7)
# global_plan(n_trials=100, planner="rrt")
# global_plan(n_trials=100, planner="rrt_star")
# global_plan(n_trials=100, planner="rrt_star_informed")