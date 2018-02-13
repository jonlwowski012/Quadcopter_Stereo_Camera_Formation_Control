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

### Read PID Results
with open('Linear/square/PID/results.csv', 'r') as fp:
	reader = csv.reader(fp)
	x1 = []
	y1 = []
	x2 = []
	y2 = []
	z = []
	zg = []
	i = 0
	for row in reader:
		i+= 1
		x1.append(float(row[0]))
		y1.append(float(row[1]))
		x2.append(float(row[2]))
		y2.append(float(row[3]))
		z.append(5.0)
		zg.append(0.0)
ox1 = []
ox2 = []
oy1 = []
oy2 = []
for i in range(len(x1)):
	ox1.append(x1[i]-1)
	ox2.append(x2[i]+1)
	oy1.append(y1[i]-1)
	oy2.append(y2[i]+1)

PID = Plotter(x1,y1,ox1,oy1,x2,y2,ox2,oy2,z,zg)


# 3D Plot of UAVs
fig = plt.figure()
ax = fig.gca(projection='3d')
### PID PLOT
ax.plot(PID.x1,PID.y1,PID.z, 'r', linewidth=5, label='Leader MAV Pose')
ax.plot(PID.x2,PID.y2,PID.z, 'c', linewidth=5, label='Follower MAV Pose')
ax.plot(PID.ox2[0:1000],PID.oy2[0:1000],PID.zg[0:1000], 'c*', markersize=12,markevery=50, label='Follower Trackpoint Pose')
ax.plot(PID.ox2[1000:],PID.oy2[1000:],PID.zg[1000:], 'c*', markersize=12,markevery=1000)
ax.plot(PID.ox1[500:],PID.oy1[500:],PID.zg[500:], 'r^', markersize=12,markevery=1000, label='Leader Trackpoint Pose')
ax.set_xlabel('X Position (m)',fontsize=30,labelpad=20)
ax.set_ylabel('Y Position (m)',fontsize=30,labelpad=20)
ax.set_zlabel('Z Position (m)',fontsize=30,labelpad=20)
ax.legend(prop={'size': 28}, loc=6)
plt.show()
