import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('local_planner_dynamic_LQR.csv', header=0)
data = np.array(df)
print(data.shape)

print(np.average(data[:,1]))

execution_time = [data[k,0] for k in range(data.shape[0]) if data[k,1]==1]
plt.boxplot(execution_time, labels=["Execution time"])
plt.legend()
plt.show()