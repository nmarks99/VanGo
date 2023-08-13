#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
import sys   

path = sys.argv[1]
data = np.loadtxt(path, delimiter=",")
t = data[:,0]

plt.style.use("ggplot")
fig, ax = plt.subplots()
ax.plot(t, data[:,1], "-b", label="Left")
ax.plot(t, data[:,2], "-r", label="Right")
ax.set(ylabel="Angle (rad)")
ax.set(xlabel="Time (ms)")
ax.legend()
plt.show()
