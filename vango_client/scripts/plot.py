#!/usr/bin/env python3
from matplotlib import pyplot as plt; plt.style.use('ggplot')
import numpy as np
import sys

file = sys.argv[1] if len(sys.argv) == 2 else None
if file is None:
    raise RuntimeError("No input file specified")

traj = np.loadtxt(file, delimiter=",", skiprows=1)
fig, ax = plt.subplots()
ax.scatter(traj[:,0], traj[:,1], color="blue")
plt.show()
