import matplotlib.pyplot as plt
import numpy as np

data = []
with open("./data.csv", "r") as f:
    for line in f:
        line = line[26:]
        line = line.split(',')
        d = [float(line[0]), float(line[1])]
        data.append(d)
data = np.array(data)
data.flatten()

fig,ax = plt.subplots()
ax.plot(data[:,1],"-b")
ax.grid()
plt.show()
