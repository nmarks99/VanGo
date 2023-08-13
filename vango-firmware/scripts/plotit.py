#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
import sys   

path = sys.argv[1]
data = np.loadtxt(path, delimiter=",")
t = np.linspace(0,1,len(data))
# t = data[:,0]

plt.style.use("ggplot")
fig, ax = plt.subplots()
ax.plot(t, data[:,0], "-b", label="Left")
ax.plot(t, data[:,1], "-r", label="Right")
ax.set(ylabel="Speed (rad/s)")
ax.set(xlabel="Time (ms)")
ax.legend()
plt.show()
