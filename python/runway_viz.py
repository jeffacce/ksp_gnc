import matplotlib.pyplot as plt
import json
from mpl_toolkits.mplot3d import Axes3D

with open('ksc_runway.json', 'r') as f:
	data = json.load(f)

x = [elem[0] for elem in data]
y = [elem[1] for elem in data]
z = [elem[2] for elem in data]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_aspect(1)
ax.plot(x, y, z)
plt.show()
