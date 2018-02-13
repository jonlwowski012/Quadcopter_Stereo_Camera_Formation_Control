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
with open('Gazebo/square/PID/error.csv', 'r') as fp:
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
error_x = []
error_y = []
error=[]
for i in range(len(x1)):
	ox1 = x1[i]+px1[i]
	ox2 = x2[i]+px2[i]
	error_x.append(ox1-ox2)
	oy1 = y1[i]+py1[i]
	oy2 = y2[i]+py2[i]
	error_y.append(oy1-oy2)
	error.append(math.sqrt(error_x[i]**2+error_y[i]**2))

PID = Plotter(x1,y1,x2,y2,z,zg,error_x,error_y,error)

### Read SFB Results
with open('Gazebo/square/SFB/error.csv', 'r') as fp:
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
error_x = []
error_y = []
error=[]
for i in range(len(x1)):
	ox1 = x1[i]+px1[i]
	ox2 = x2[i]+px2[i]
	error_x.append(ox1-ox2)
	oy1 = y1[i]+py1[i]
	oy2 = y2[i]+py2[i]
	error_y.append(oy1-oy2)
	error.append(math.sqrt(error_x[i]**2+error_y[i]**2))

SFB = Plotter(x1,y1,x2,y2,z,zg,error_x,error_y,error)

### Read Bang Results
with open('Gazebo/square/Bang/error.csv', 'r') as fp:
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
error_x = []
error_y = []
error=[]
for i in range(len(x1)):
	ox1 = x1[i]+px1[i]
	ox2 = x2[i]+px2[i]
	error_x.append(ox1-ox2)
	oy1 = y1[i]+py1[i]
	oy2 = y2[i]+py2[i]
	error_y.append(oy1-oy2)
	error.append(math.sqrt(error_x[i]**2+error_y[i]**2))

Bang = Plotter(x1,y1,x2,y2,z,zg,error_x,error_y,error)

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
error_x = []
error_y = []
error=[]
for i in range(len(x1)):
	ox1 = x1[i]+px1[i]
	ox2 = x2[i]+px2[i]
	error_x.append(ox1-ox2)
	oy1 = y1[i]+py1[i]
	oy2 = y2[i]+py2[i]
	error_y.append(oy1-oy2)
	error.append(math.sqrt(error_x[i]**2+error_y[i]**2))

MPCL = Plotter(x1,y1,x2,y2,z,zg,error_x,error_y,error)


### Read MPC Small Results
with open('Gazebo/square/MPC/small_error.csv', 'r') as fp:
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
error_x = []
error_y = []
error=[]
for i in range(len(x1)):
	ox1 = x1[i]+px1[i]
	ox2 = x2[i]+px2[i]
	error_x.append(ox1-ox2)
	oy1 = y1[i]+py1[i]
	oy2 = y2[i]+py2[i]
	error_y.append(oy1-oy2)
	error.append(math.sqrt(error_x[i]**2+error_y[i]**2))

MPCS = Plotter(x1,y1,x2,y2,z,zg,error_x,error_y,error)

### Plot
# Plot Error
fig,ax = plt.subplots(1)
plt.axis([25000,75000,-2.5,2.5])
plt.plot(MPCL.error_x[0:75000],'c-',linewidth=3, label='LWMPC')
plt.plot(MPCS.error_x[0:75000],'m-',linewidth=3, label='SWMPC')
plt.plot(Bang.error_x[0:75000],'b',linewidth=3, label='Bang',markevery=1)
plt.plot(PID.error_x[0:75000],'r',linewidth=3, label='PID',markevery=1)
plt.plot(SFB.error_x[0:75000],'g',linewidth=3, label='SFB',markevery=1)
plt.title("Euclidean Error of Points on the Ground",fontsize=25,y=1.02)
plt.xlabel('Timestep',fontsize=20)
plt.ylabel('Euclidean Error (m)',fontsize=20)
plt.xticks(np.arange(25000,75010, 15000.0),size=20)
'''ax.add_patch(
    patches.Rectangle(
        (25000, -2.5),
        50000,
        5,
        fill=False, linewidth=3,edgecolor='k',linestyle='--'      # remove background
    )
)'''
plt.legend(prop={'size': 18},loc=1)
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
