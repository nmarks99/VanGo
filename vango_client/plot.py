from matplotlib import pyplot as plt; plt.style.use('ggplot')
import numpy as np

traj = np.loadtxt("./data.csv", delimiter=",")
fig, ax = plt.subplots()
ax.scatter(traj[:,0], traj[:,1], color="blue")
plt.show()
