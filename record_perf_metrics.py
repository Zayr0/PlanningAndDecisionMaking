"""here performance metrics are compares"""
from simulation import run
import pandas as pd


def different_global_planners_static():
    experiment_name = "different_global_planners_static"
    n_trials = 3

    for _ in range(n_trials):
        try:
            results = run(auto_testing=True)
            results.append(1)
        except:
            results = [None, None, 0]
        columns = ['Solver Time', 'Execution Time', 'Success']
        filename = experiment_name + ".csv"
        try:
            df = pd.read_csv(filename, header=0, usecols=columns)
        except:
            df = pd.DataFrame(columns=columns)
        df.loc[len(df)] = results
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
    settings = {"env_static": False, "env_dynamic": True, "dynamic_controller": "LQR"}
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

local_planner_dynamic_MPC(n_trials=100)
local_planner_dynamic_LQR(n_trials=100)