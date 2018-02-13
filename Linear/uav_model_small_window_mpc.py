import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import time
import numpy as np
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
		self.Ox = [Ux+Px]
		self.Oy = [Uy+Py]
		self.Oz = [Uz+Pz]


### Make Master UAV
master = UAV(Ux=2.,Ux_dot=0.0,Px=-1.,Uy=2.,Uy_dot=0.0,Py=1.0,Uz=1.0,Uz_dot=0.0,Pz=-1.0)


### Make Slave UAV
slave = UAV(Ux=-10.,Ux_dot=0.0,Px=1.,Uy=-10.,Uy_dot=0.0,Py=1.0,Uz=1.0,Uz_dot=0.0,Pz=-1.0)

### Simulation Parameters
ts = 0.01
dt = 0.01
max_vel = 1.0
count = [0]
phi = np.matrix([[0.,0.,1.,0,1,0],[0,0,0,1,0,1],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
gam = np.matrix([[dt,0],[0.,dt],[dt,0],[0,dt],[0,0],[0,0]])
window_size = 10
Q = np.matrix([[10000., 0, 0, 0, 0, 0],[0, 100000., 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0]])
R = np.matrix([[0.0001, 0],[0,0.0001]])
index = 0
p_x = 1
p_y = 1
counter = 0
its = 50000

### Calculate K States
master_states1 = np.matrix([[master.Ox[0]],[master.Oy[0]],[master.Ux[0]],[master.Uy[0]],[master.Px],[master.Py]])
slave_states = np.matrix([[slave.Ox[0]],[slave.Oy[0]],[slave.Ux[0]],[slave.Uy[0]],[slave.Px],[slave.Py]])
sin_in = 0

for k in range(0,its):

	#### Slave 1 Controller
	if (k % window_size == 0) or (k == 0):
		u_mpc = [];
		#Solve for P
		P = [None] * window_size;
		P[window_size-1] = np.matrix([[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]);
		for i in range(window_size-2, 0, -1):
			a = phi.transpose()*P[i+1]*phi
			b = phi.transpose()*P[i+1]*gam
			c = np.linalg.inv(R+gam.transpose()*P[i+1]*gam)
			d = gam.transpose()*P[i+1]*phi
			P[i]= a-b*c*d+Q;

		# Solve for M
		M = [None]*(window_size-1)
		for i in range(0,window_size-1):
			a = np.linalg.inv(R+gam.transpose()*P[i+1]*gam);
			b = gam.transpose()*P[i+1]*phi;
			M[i]=-(a*b);
				
	r=np.matrix([[master_states1[0,0]],[master_states1[1,0]],[master_states1[2,0]+slave_states[4,0]-master_states1[4,0]],[master_states1[3,0]+slave_states[5,0]-master_states1[5,0]],[slave_states[4,0]],[slave_states[5,0]]])
	Us = M[index]*(slave_states-r)
	slave_vel_x=Us[0,0]
	slave_vel_y=Us[1,0]

	index = index + 1;
	if index > window_size-2:
		index = 1
	#print Us,slave_vel_x,slave_vel_y

	# Calculate for X
	master.Ux.append(master.Ux[k]+(master.Ux_dot*ts))
	slave.Ux_dot = slave_vel_x
	if slave.Ux_dot > max_vel:
		slave.Ux_dot = max_vel
	elif slave.Ux_dot < -max_vel:
		slave.Ux_dot = -max_vel


	slave.Ux.append(slave.Ux[k]+(slave.Ux_dot*ts))
	master.Ox.append(master.Ux[k]+(master.Ux_dot*ts)+master.Px)
	slave.Ox.append(slave.Ux[k]+(slave.Ux_dot*ts)+slave.Px)

	# Calculate for y
	master.Uy.append(master.Uy[k]+(master.Uy_dot*ts))
	slave.Uy_dot = slave_vel_y
	if slave.Uy_dot > max_vel:
		slave.Uy_dot = max_vel
	elif slave.Uy_dot < -max_vel:
		slave.Uy_dot = -max_vel
	#print k
	slave.Uy.append(slave.Uy[k]+(slave.Uy_dot*ts))
	master.Oy.append(master.Uy[k]+(master.Uy_dot*ts)+master.Py)
	slave.Oy.append(slave.Uy[k]+(slave.Uy_dot*ts)+slave.Py)

	# Calculate for z
	master.Uz.append(master.Uz[k]+(master.Uz_dot*ts))
	master.Uz_dot = 0.0
	slave.Uy_dot = 0.0
	slave.Uz.append(slave.Uz[k]+(slave.Uz_dot*ts))
	master.Oz.append(master.Uz[k]+(master.Uz_dot*ts)+master.Pz)
	slave.Oz.append(slave.Uz[k]+(slave.Uz_dot*ts)+slave.Pz)
	count.append(k)
	#print Ux1[k], Ux1_dot

	### Calculate K States
	master_states1 = np.matrix([[master.Ox[k+1]],[master.Oy[k+1]],[master.Ux[k+1]],[master.Uy[k+1]],[master.Px],[master.Py]])

	slave_states = np.matrix([[slave.Ox[k+1]],[slave.Oy[k+1]],[slave.Ux[k+1]],[slave.Uy[k+1]],[slave.Px],[slave.Py]])
	
	if counter == 0:
		master.Ux_dot = 0.3
		master.Uy_dot = 0.0
	if counter== 8000:
		master.Ux_dot = 0.0
		master.Uy_dot = 0.3
	if counter == 16000:
		master.Ux_dot = -0.3
		master.Uy_dot = 0.0
	if counter == 24000:
		master.Ux_dot = 0.0
		master.Uy_dot = -0.3
	if counter == 32000:
		counter = -1
	counter += 1

	'''master.Ux_dot = 0.5*math.sin(sin_in)
	master.Uy_dot = 0.5*math.sin(sin_in+3.14/2)
	sin_in += 0.0001
	if sin_in >= 2*3.14:
		sin_in = 0.'''

	with open('/home/ace/catkin_ws/src/Quadcopter_Stereo_Camera_Formation_Control/results/MPC/small_results.csv', 'a') as fp:
				r_writer = csv.writer(fp, delimiter=',')
				r_writer.writerow([master.Ux[k],master.Uy[k],slave.Ux[k],slave.Uy[k]])
	

### X Plotting Stuff
remove_amount = 1
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

# Plot Y Poses
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
