import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

### Plotting Class
class Plotter:
	def __init__(self,x1,y1,ox1,oy1,x2,y2,ox2,oxy2,z,zg):
		self.x1 = x1
		self.y1 = y1
		self.ox1 = ox1
		self.oy1 = oy1
		self.x2 = x2
		self.y2 = y2
		self.ox2 = ox2
		self.oy2 = oy2
		self.z = z
		self.zg = zg


### Read MPC Large Results
with open('Gazebo/square/MPC/large_error.csv', 'r') as fp:
	reader = csv.reader(fp)
	px1 = []
	py1 = []
	x1 = []
	y1 = []
	px2 = []
	py2 = []
	x2 = []
	y2 = []
	z = []
	zg = []
	i = 0
	for row in reader:
		i+= 1
		x1.append(float(row[0]))
		x2.append(float(row[1]))
		px1.append(float(row[2]))
		px2.append(float(row[3]))
		y1.append(float(row[4]))
		y2.append(float(row[5]))
		py1.append(float(row[6]))
		py2.append(float(row[7]))
		z.append(5.0)
		zg.append(0.0)
ox1 = []
ox2 = []
oy1 = []
oy2 = []
for i in range(len(x1)):
	ox1.append(x1[i]+px1[i])
	ox2.append(x2[i]+px2[i])
	oy1.append(y1[i]+py1[i])
	oy2.append(y2[i]+py2[i])

MPCL = Plotter(x1,y1,ox1,oy1,x2,y2,ox2,oy2,z,zg)


### Plot
# Plot Error
# 3D Plot of UAVs
fig = plt.figure()
ax = fig.gca(projection='3d')
### PID PLOT
ax.plot(MPCL.x1[10000:70000],MPCL.y1[10000:70000],MPCL.z[10000:70000], 'r', linewidth=5, label='Leader MAV Pose')
ax.plot(MPCL.x2[10000:70000],MPCL.y2[10000:70000],MPCL.z[10000:70000], 'c', linewidth=5, label='Follower MAV Pose')
ax.plot(MPCL.ox2[10000:15000],MPCL.oy2[10000:15000],MPCL.zg[10000:15000], 'c*', markersize=12,markevery=400, label='Follower Trackpoint Pose')
ax.plot(MPCL.ox2[15000:70000],MPCL.oy2[15000:70000],MPCL.zg[15000:70000], 'c*', markersize=12,markevery=3000)
ax.plot(MPCL.ox1[15000:70000],MPCL.oy1[15000:70000],MPCL.zg[15000:70000], 'r^', markersize=12,markevery=3000, label='Leader Trackpoint Pose')
ax.set_xlabel('X Position (m)',fontsize=30,labelpad=20)
ax.set_ylabel('Y Position (m)',fontsize=30,labelpad=20)
ax.set_zlabel('Z Position (m)',fontsize=30,labelpad=20)
ax.legend(prop={'size': 28}, loc=6)
plt.show()

