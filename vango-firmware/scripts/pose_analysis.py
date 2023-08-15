#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt

# time (ms), left_angle, right_angle, theta, x, y
data = np.loadtxt("./pose_data.csv",delimiter=",", skiprows=1)
t = data[:,0]
t = (t - t[0]) / 1000.0 # convert to seconds
left_angle = data[:,1]
right_angle = data[:,2]
theta = data[:,3]
x = data[:,4]
y = data[:,5]

plt.style.use('ggplot')
fig, ax = plt.subplots()
ax.plot(t, x, label="x")
ax.plot(t, y, label="y")
ax.plot(t, theta, label="theta")
ax.set(xlabel="Time (s)", ylabel="Position (m)")
ax.set(
    title=r"Pose estimate with commanded twist $(\dot{\theta}, \dot{x}, \dot{y}) = (0,0.2,0)$"
)
ax.set_xticks(np.arange(0,10.5,0.5))
ax.set_yticks(np.arange(0,2.1,0.1))
ax.legend()
plt.show()
