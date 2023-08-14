#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
import sys   

path = sys.argv[1]
data = np.loadtxt(path, delimiter=",")
t = data[:,0]
t = (t - t[0]) / 1000
# t = np.linspace(0,len(data)/100,len(data))

plt.style.use("ggplot")
fig, ax = plt.subplots()
ax.plot(t[480:600], data[:,1][480:600], "-b", label="Left")
# ax.plot(t, data[:,2], "-r", label="Right")
ax.set(ylabel="Speed (rad/s)")
ax.set(xlabel="Time (s)")
ax.set(title="Kp=0.3, low-pass filtered")
ax.set_xticks(np.arange(4.7,6.3,0.05))
ax.legend()
plt.show()
