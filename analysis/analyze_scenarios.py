import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


csv_path = '../data_human_takeover_2.csv'

df = pd.read_csv(csv_path)
t = df['t'].to_numpy()
xm = df['Xm'].to_numpy()
xo = df['Xo'].to_numpy()

fig, (a1, a2) = plt.subplots(nrows=2,ncols=1)

a1.plot(t, xm)
a2.plot(t, xo)
plt.show()
