#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
import sys   

path = sys.argv[1]
data = np.loadtxt(path, delimiter=",")
t = np.array([100 * i for i in list(range(0,len(data)))]) / 1000 # sec

plt.style.use("ggplot")
fig, ax = plt.subplots()
ax.plot(t, data[:,0], "-b", label="Left motor")
ax.plot(t, data[:,1], "-r", label="Right motor")
ax.set_xticks(np.arange(0,t[-1]+1, 1))
ax.set(ylabel="Speed (rad/s)")
ax.set(xlabel="Time (sec)")
ax.set(title="$K_p=0.3$, $K_i=0$, $K_d=0$")
ax.legend()
plt.show()
