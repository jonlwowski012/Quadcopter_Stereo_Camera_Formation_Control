import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import time
import csv
import math

class UAV:
	def __init__(self, Ux, Ux_dot, Px, Uy, Uy_dot, Py, Uz, Uz_dot, Pz):
		self.Ux = [Ux]
		self.Ux_dot = Ux_dot
		self.Px = Px
		self.Uy = [Uy]
		self.Uy_dot = Uy_dot
		self.Py = Py
		self.Uz = [Uz]
		self.Uz_dot = Uz_dot
		self.Pz = Pz
		self.Ox = []
		self.Oy = []
		self.Oz = []


### Make Master UAV
master = UAV(Ux=2.,Ux_dot=0.0,Px=-1.,Uy=2.,Uy_dot=0.0,Py=-1.0,Uz=1.0,Uz_dot=0.0,Pz=-1.0)


### Make Slave UAV
slave = UAV(Ux=-10.,Ux_dot=0.0,Px=1.,Uy=-10.,Uy_dot=0.0,Py=1.0,Uz=1.0,Uz_dot=0.0,Pz=-1.0)

### Simulation Parameters
ts = 0.01
max_vel = 1.0
count = [0]

p_x = 1
p_y = 1
counter = 0
its = 50000
sin_in = 0

for k in range(1,its):
	# Calculate for X
	master.Ux.append(master.Ux[k-1]+(master.Ux_dot*ts))
	slave.Ux_dot = p_x*((master.Ux[k-1]/ts)-(slave.Ux[k-1]/ts)+(master.Ux_dot)+(master.Px/ts)-(slave.Px/ts))
	if slave.Ux_dot > max_vel:
		slave.Ux_dot = max_vel
	elif slave.Ux_dot < -max_vel:
		slave.Ux_dot = -max_vel


	slave.Ux.append(slave.Ux[k-1]+(slave.Ux_dot*ts))
	master.Ox.append(master.Ux[k-1]+(master.Ux_dot*ts)+master.Px)
	slave.Ox.append(slave.Ux[k-1]+(slave.Ux_dot*ts)+slave.Px)

	# Calculate for y
	master.Uy.append(master.Uy[k-1]+(master.Uy_dot*ts))
	slave.Uy_dot = p_y*((master.Uy[k-1]/ts)-(slave.Uy[k-1]/ts)+(master.Uy_dot)+(master.Py/ts)-(slave.Py/ts))
	if slave.Uy_dot > max_vel:
		slave.Uy_dot = max_vel
	elif slave.Uy_dot < -max_vel:
		slave.Uy_dot = -max_vel
	#print k
	slave.Uy.append(slave.Uy[k-1]+(slave.Uy_dot*ts))
	master.Oy.append(master.Uy[k-1]+(master.Uy_dot*ts)+master.Py)
	slave.Oy.append(slave.Uy[k-1]+(slave.Uy_dot*ts)+slave.Py)

	# Calculate for z
	master.Uz.append(master.Uz[k-1]+(master.Uz_dot*ts))
	master.Uz_dot = 0.0
	slave.Uy_dot = 0.0
	slave.Uz.append(slave.Uz[k-1]+(slave.Uz_dot*ts))
	master.Oz.append(master.Uz[k-1]+(master.Uz_dot*ts)+master.Pz)
	slave.Oz.append(slave.Uz[k-1]+(slave.Uz_dot*ts)+slave.Pz)
	count.append(k)
	#print Ux1[k], Ux1_dot
	
	'''if counter == 0:
		master.Ux_dot = 0
		master.Uy_dot = 0.0
	if counter== 8000:
		master.Ux_dot = 0.0
		master.Uy_dot = 0
	if counter == 16000:
		master.Ux_dot = 0.
		master.Uy_dot = 0.0
	if counter == 24000:
		master.Ux_dot = 0.0
		master.Uy_dot = -0.
	if counter == 32000:
		counter = -1
	counter += 1'''

	master.Ux_dot = 0.5*math.sin(sin_in)
	master.Uy_dot = 0.5*math.sin(sin_in+3.14/2)
	sin_in += 0.0001
	if sin_in >= 2*3.14:
		sin_in = 0.

	with open('/home/ace/catkin_ws/src/Quadcopter_Stereo_Camera_Formation_Control/results/SFB/results.csv', 'a') as fp:
				r_writer = csv.writer(fp, delimiter=',')
				r_writer.writerow([master.Ux[k],master.Uy[k],slave.Ux[k],slave.Uy[k]])
	

### X Plotting Stuff
remove_amount = 4000
'''plt.figure()
plt.plot(count,Ux1)
plt.plot(count,Ux2)
plt.title("Actual Poses of UAVs (X)")'''
plt.figure()
plt.title("Actual Poses of Points on Ground (X)")
'''# Rise Time X Poses
Rt_index = next(x[0] for x in enumerate(slave.Ox) if x[1] > 0.999)
Rt = ts*(Rt_index+1)
plt.scatter(Rt_index,slave.Ox[Rt_index],c='g')
plt.scatter(0,slave.Ox[Rt_index],c='g')
plt.plot([0, Rt_index],[slave.Ox[Rt_index],slave.Ox[Rt_index]],'g')
plt.text(0, slave.Ox[Rt_index]+.01, 'Rt=Pt=' + str(Rt) + 'secs', fontsize=10, color='g')
# Overshoot X Poses
Os = max(slave.Ox)
Os_actual = master.Ox[0]-Os
plt.text(Rt_index+100, slave.Ox[Rt_index]+.01, 'Overshoot=' + str(int(Os_actual)), fontsize=10, color='g')'''

'''#Delay Time X
Dt_index = next(x[0] for x in enumerate(slave.Ox) if x[1] > 0.5)
Dt = ts*(Dt_index+1)
plt.scatter(Dt_index,slave.Ox[Dt_index],c='g')
plt.scatter(0,slave.Ox[Dt_index],c='g')
plt.plot([0, Dt_index],[slave.Ox[Dt_index],slave.Ox[Dt_index]],'g')
plt.text(0, slave.Ox[Dt_index]+.01, 'Dt=' + str(Dt) + 'secs', fontsize=10, color='g')'''

# Plot X Poses
plt.plot(slave.Ox,'b')
plt.plot(master.Ox,'r--')

# Error Plot
plt.figure()
plt.title("Error of Points on the Ground (X)")
plt.plot([a-b for a,b in zip(master.Ox[:-10*remove_amount],slave.Ox[:-10*remove_amount])])

### Y Plotting Stuff
'''plt.figure()
plt.plot(count,Uy1)
plt.plot(count,Uy2)
plt.title("Actual Poses of UAVs (X)")'''
plt.figure()
plt.title("Actual Poses of Points on Ground (Y)")
'''# Rise Time Y Poses
Rt_index = next(x[0] for x in enumerate(slave.Oy) if x[1] > 0.999)
Rt = ts*(Rt_index+1)
plt.scatter(Rt_index,slave.Oy[Rt_index],c='g')
plt.scatter(0,slave.Oy[Rt_index],c='g')
plt.plot([0, Rt_index],[slave.Oy[Rt_index],slave.Oy[Rt_index]],'g')
plt.text(0, slave.Oy[Rt_index]+.01, 'Rt=Pt=' + str(Rt) + 'secs', fontsize=10, color='g')
# Overshoot Y Poses
Os = max(slave.Oy)
Os_actual = master.Oy[0]-Os
plt.text(Rt_index+100, slave.Oy[Rt_index]+.01, 'Overshoot=' + str(int(Os_actual)), fontsize=10, color='g')

#Delay Time Y
Dt_index = next(x[0] for x in enumerate(slave.Oy) if x[1] > 0.5)
Dt = ts*(Dt_index+1)
plt.scatter(Dt_index,slave.Oy[Dt_index],c='g')
plt.scatter(0,slave.Oy[Dt_index],c='g')
plt.plot([0, Dt_index],[slave.Oy[Dt_index],slave.Oy[Dt_index]],'g')
plt.text(0, slave.Oy[Dt_index]+.01, 'Dt=' + str(Dt) + 'secs', fontsize=10, color='g')'''

# Plot X Poses
plt.plot(slave.Oy,'b')
plt.plot(master.Oy,'r--')

# Error Plot
plt.figure()
plt.title("Error of Points on the Ground (Y)")
plt.plot([a-b for a,b in zip(master.Oy[:-10*remove_amount],slave.Oy[:-10*remove_amount])])

'''# Plot Z Poses
plt.figure()
plt.plot(count,Uz1)
plt.plot(count,Uz2)
plt.title("Actual Poses of UAVs (Z)")
plt.figure()
plt.title("Actual Poses of Points on Ground (Z)")
plt.plot(Oz1)
plt.plot(Oz2)'''

# 3D Plot of UAVs
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(master.Ux,master.Uy,master.Uz, 'r', label='Leader MAV Pose')
ax.plot(slave.Ux,slave.Uy,slave.Uz, 'b', label='Follower MAV Pose')
ax.plot(slave.Ox,slave.Oy,slave.Oz, 'm--', linewidth=6, label='Leader Trackpoint Pose')
ax.plot(master.Ox,master.Oy,master.Oz, 'c--', markersize=2, label='Follower Trackpoint Pose')

ax.legend(prop={'size': 20})
plt.show()
