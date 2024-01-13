import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('metrics.csv', header=0)
data = np.array(df)
plt.boxplot(data, labels=["Planner 1", "Planner 2"])
plt.legend()
plt.show()
