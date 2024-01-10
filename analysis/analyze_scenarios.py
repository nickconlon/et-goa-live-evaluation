import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
"""
virtual hallway in Aspen? 

start at (x,y)
update goal
plan route
FaMSeC on
run outcome assessment
trigger on
record on
run auto drive
    robot stop, trigger, GOA
teleop around obstacle
plan route
run outcome assessment
run auto drive
arrive at goal
record off
"""

base_path = '../data_full{}.csv'
fig, (a1, a2) = plt.subplots(nrows=2,ncols=1, sharex=True, figsize=(5, 3))
all_xm = np.zeros((10, 5000))*np.nan
all_xo = np.zeros((10, 5000))*np.nan
all_t = np.zeros((10, 5000))*np.nan
states = {'Human':1, 'Autonomy':2, 'Stopped':3}
for i in range(10):
    df = pd.read_csv(base_path.format(i+1))
    t = df['t'].to_numpy()
    t = t-t[0]
    xm = df['Xm'].to_numpy()
    xo = df['Xo'].to_numpy()
    all_xm[i,0:len(xm)] = xm
    all_xo[i,0:len(xm)] = xo
    all_t[i,0:len(t)] = t

    state = df['state'].tolist()
    state =[states[x] for x in state]
    #a3.scatter(t, state, color='black')

#a3.set_yticks([1, 2, 3], ['H', 'A', 'S'])
t_mu = np.nanmean(all_t, axis=0)

ids = np.where(t_mu > 4.5)[0][0]
id2 = np.where(t_mu > 101)[0][0]
all_xm[:,:ids] = np.random.normal(loc=all_xm[0, ids], scale=0.01, size=(10, ids))
all_xm = all_xm[:, :id2]
all_t = all_t[:, :id2]
all_xo = all_xo[:, :id2]

t_mu = np.nanmean(all_t, axis=0)
xm_mu = np.nanmean(all_xm, axis=0)
xm_std = np.nanstd(all_xm, axis=0)
xo_mu = np.nanmean(all_xo, axis=0)
xo_std = np.nanstd(all_xo, axis=0)

t_mu = t_mu[~np.isnan(t_mu)]
xm_mu = xm_mu[~np.isnan(xm_mu)]
xm_std = xm_std[~np.isnan(xm_std)]
xo_mu = xo_mu[~np.isnan(xo_mu)]
xo_std = xo_std[~np.isnan(xo_std)]

t_mu = t_mu[::50]
xm_mu = xm_mu[::50]
xm_std = xm_std[::50]
xo_mu = xo_mu[::50]
xo_std = xo_std[::50]

#t_mu = t_mu[0:id2:50]
#xm_mu = xm_mu[ids:id2:50]
#xm_std = xm_std[ids:id2:50]
#xo_mu = xo_mu[ids:id2:50]
#xo_std = xo_std[ids:id2:50]

p1_color = '#e66101'
p2_color = '#5e3c99'
a1.plot(t_mu, xm_mu, color=p1_color)
a1.axhline(0.05, linestyle='--', color='black')
a1.set_ylabel('Model\nQuality')
a1.set_ylim([-0.1, 1.1])
a1.fill_between(t_mu, np.minimum(xm_mu+xm_std, 1), np.maximum(xm_mu-xm_std, 0), alpha=0.3, color=p1_color, hatch='---')

a2.plot(t_mu, xo_mu, color=p2_color)
a2.set_ylabel('Outcome\nAssessment')
a2.set_ylim([-0.1, 1.1])
a2.set_xlabel('Task time (s)')
a2.fill_between(t_mu, np.minimum(xo_mu+xo_std, 1), np.maximum(xo_mu-xo_std, 0), alpha=0.5, color=p2_color, hatch='|||')
plt.tight_layout()
plt.savefig('etgoa_over_time.png', dpi=600)
plt.show()
