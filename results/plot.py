import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import matplotlib.patches as patches

### Plotting Class
class Plotter:
	def __init__(self,x1,y1,x2,y2,z,zg,error_x,error_y,error):
		self.x1 = x1
		self.y1 = y1
		self.x2 = x2
		self.y2 = y2
		self.z = z
		self.zg = zg
		self.error_x = error_x
		self.error_y = error_y
		self.error = error

### Read PID Results
with open('Nonlinear/square/PID/results.csv', 'r') as fp:
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
error_x = []
error_y = []
error=[]
for i in range(len(x1)):
	ox1 = x1[i]-1
	ox2 = x2[i]+1
	error_x.append(ox1-ox2)
	oy1 = y1[i]-1
	oy2 = y2[i]+1
	error_y.append(oy1-oy2)
	error.append(math.sqrt(error_x[i]**2+error_y[i]**2))

PID = Plotter(x1,y1,x2,y2,z,zg,error_x,error_y,error)

### Read SFB Results
with open('Nonlinear/square/SFB/results.csv', 'r') as fp:
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
error_x = []
error_y = []
error=[]
for i in range(len(x1)):
	ox1 = x1[i]-1
	ox2 = x2[i]+1
	error_x.append(ox1-ox2)
	oy1 = y1[i]-1
	oy2 = y2[i]+1
	error_y.append(oy1-oy2)
	error.append(math.sqrt(error_x[i]**2+error_y[i]**2))

SFB = Plotter(x1,y1,x2,y2,z,zg,error_x,error_y,error)

### Read Bang Results
with open('Nonlinear/square/Bang/results.csv', 'r') as fp:
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
error_x = []
error_y = []
error=[]
for i in range(len(x1)):
	ox1 = x1[i]+1
	ox2 = x2[i]-1
	error_x.append(ox1-ox2)
	oy1 = y1[i]+1
	oy2 = y2[i]-1
	error_y.append(oy1-oy2)
	error.append(math.sqrt(error_x[i]**2+error_y[i]**2))

Bang = Plotter(x1,y1,x2,y2,z,zg,error_x,error_y,error)

### Read MPC Large Results
with open('Nonlinear/square/MPC/large_window_results.csv', 'r') as fp:
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
error_x = []
error_y = []
error=[]
for i in range(len(x1)):
	ox1 = x1[i]+1
	ox2 = x2[i]-1
	error_x.append(ox1-ox2)
	oy1 = y1[i]+1
	oy2 = y2[i]-1
	error_y.append(oy1-oy2)
	error.append(math.sqrt(error_x[i]**2+error_y[i]**2))

MPCL = Plotter(x1,y1,x2,y2,z,zg,error_x,error_y,error)


### Read MPC Small Results
with open('Nonlinear/square/MPC/small_window_results.csv', 'r') as fp:
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
error_x = []
error_y = []
error=[]
for i in range(len(x1)):
	ox1 = x1[i]+1
	ox2 = x2[i]-1
	error_x.append(ox1-ox2)
	oy1 = y1[i]+1
	oy2 = y2[i]-1
	error_y.append(oy1-oy2)
	error.append(math.sqrt(error_x[i]**2+error_y[i]**2))

MPCS = Plotter(x1,y1,x2,y2,z,zg,error_x,error_y,error)

### Plot
# Plot Error
fig,ax = plt.subplots(1)
plt.plot(MPCL.error_x[0:1500],'c-',linewidth=3, label='LWMPC')
plt.plot(MPCS.error_x[0:1500],'m-',linewidth=3, label='SWMPC')
plt.plot(Bang.error_x[0:1500],'b',linewidth=3, label='Bang',markevery=1)
plt.plot(PID.error_x[0:1500],'r--',linewidth=3, label='PID',markevery=1)
plt.plot(SFB.error_x[0:1500],'g--',linewidth=3, label='SFB',markevery=1)
plt.title("Euclidean Error of Points on the Ground",fontsize=25,y=1.02)
plt.xlabel('Timestep',fontsize=20)
plt.ylabel('Euclidean Error (m)',fontsize=20)
plt.xticks(np.arange(0,1700, 300.0),size=20)


ax.add_patch(
    patches.Rectangle(
        (900, -0.7),
        300,
        1.4,
        fill=False, linewidth=3,edgecolor='r',linestyle='--'      # remove background
    )
)


plt.legend(prop={'size': 18},loc=3)
'''# Plot Error
fig = plt.figure()
plt.plot(PID.error_y[0:1000],'r-',linewidth=3)
plt.plot(SFB.error_y[0:1000],'g-',linewidth=3)
plt.plot(Bang.error_y[0:1000],'b-',linewidth=3)
plt.title("Error of Points on the Ground",fontsize=25)
plt.xlabel('Timestep',fontsize=20)
plt.ylabel('Error (m)',fontsize=20)

# 3D Plot of UAVs
fig = plt.figure()
ax = fig.gca(projection='3d')
### PID PLOT
ax.plot(PID.x1,PID.y1,PID.z, 'r', linewidth=5, label='Leader MAV Pose')
ax.plot(PID.x2,PID.y2,PID.z, 'c', linewidth=5, label='Follower MAV Pose')
ax.plot([x+1 for x in PID.x2[0:100]],[x+0 for x in PID.y2[0:100]],PID.zg[0:100], 'c*', markersize=12,markevery=1, label='Follower Trackpoint Pose')
ax.plot([x+1 for x in PID.x2[100:]],[x+0 for x in PID.y2[100:]],PID.zg[100:], 'c*', markersize=12,markevery=50)
ax.plot([x-1 for x in PID.x1],[x+0 for x in PID.y1],PID.zg, 'r^', markersize=12,markevery=50, label='Leader Trackpoint Pose')
ax.set_xlabel('X Position (m)',fontsize=30,labelpad=20)
ax.set_ylabel('Y Position (m)',fontsize=30,labelpad=20)
ax.set_zlabel('Z Position (m)',fontsize=30,labelpad=20)
ax.legend(prop={'size': 28})'''
plt.show()
