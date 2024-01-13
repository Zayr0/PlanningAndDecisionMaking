import time
import pandas as pd
import numpy as np

for _ in range(10):
    time_stamp_start = time.perf_counter()
    for _ in range(10):
        time.sleep(np.random.uniform(low=0.01, high=0.2))
    time_stamp_1 = time.perf_counter()
    n = 0
    for _ in range(1000):
        n += 1
    time_stamp_2 = time.perf_counter()

    time_stamp_end = time.perf_counter()

    # print(f"total script took {time_stamp_end-time_stamp_start}")
    # print(f"first loop took {time_stamp_1-time_stamp_start}")
    # print(f"second loop took {time_stamp_2-time_stamp_1}")
    # print(time_stamp_start)
    new_results = [time_stamp_1-time_stamp_start, time_stamp_2-time_stamp_1]


    columns = ['Solver Time', 'Execution Time']
    try:
        df = pd.read_csv('metrics.csv', header=0, usecols=columns)
    except:
        df = pd.DataFrame(columns=columns)
    print(df.columns)
    print(new_results)
    df.loc[len(df)] = new_results
    #df = df.concat(dict(zip(df.columns, metrics)), ignore_index=True)
    df.to_csv("metrics.csv", index=False)