import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('data.txt')

theta = df['theta'].to_numpy(dtype=np.float32)
speed = df['speed'].to_numpy(dtype=np.float32)

plt.plot(theta)
plt.plot(speed)
plt.show()