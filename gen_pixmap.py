import matplotlib.pyplot as plt
import numpy as np


fig, ax = plt.subplots(figsize=(3, 3))

xs, ys = np.random.rand(4)*3.9, np.random.rand(4)*3.9
plt.plot(xs, ys, color='black')
plt.scatter(xs, ys, color='black')
plt.ylim([0, 4])
plt.xlim([0, 4])
plt.yticks([0, 4])
plt.xticks([0, 4])
plt.savefig('base_map.png')
plt.show()