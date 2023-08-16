import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

df = pd.read_csv('pose_turns.csv')

xpos = df['x'].to_numpy()[::25]
ypos = df['y'].to_numpy()[::25]
tsecs = df['rosbagTimestamp'].to_numpy()[::25]// 10**9

datetime_obj1=datetime.fromtimestamp(tsecs[0])
print("The datetime object is:")
print(datetime_obj1)

datetime_obj2=datetime.fromtimestamp(tsecs[-1])
print("The datetime object is:")
print(datetime_obj2)

initial_time = tsecs[0]
tsecs = np.array([x-initial_time for x in tsecs])

vel = []
for idx in np.arange(0, len(xpos)-1):
    x1 = xpos[idx]
    x2 = xpos[idx+1]
    y1 = ypos[idx]
    y2 = ypos[idx+1]
    t1 = tsecs[idx]
    t2 = tsecs[idx+1]
    dx = np.linalg.norm(np.array([x2, y2]) - np.array([x1, y1]))
    dt = abs(t2-t1)
    if dt > 0:
        v = dx/dt
        vel.append(v)
print(xpos[0], ypos[0])
print(xpos[-1], ypos[-1])

#plt.plot(np.arange(len(vel)), vel)
plt.plot(xpos, ypos)
plt.ylim([-5, 5])
plt.xlim([-5, 5])
plt.show()
print(pd)
