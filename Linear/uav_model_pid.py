import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import time
import csv
import math

Ux1 = [2.]
Ux1_dot = 0.0
Px1 = -1.
Uy1 = [2.]
Uy1_dot = 0.0
Py1 = 0.
Uz1 = [1.]
Uz1_dot = 0.0
Pz1 = -1.

Ux2 = [-10.]
Ux2_dot = 0.0
Px2 = 1.
Uy2 = [-10.]
Uy2_dot = 0.0
Py2 = 0.
Uz2 = [1.]
Uz2_dot = 0.0
Pz2 = -1.

ts = 0.01
max_vel = 1.0

Ox1 = []
Ox2 = []
Oy1 = []
Oy2 = []
Oz1 = []
Oz2 = []
count = [0]

# X PID Constants
Kpx = 200.
Kdx = 0.1
Kix = 0.1
I_x = 0
I_minx = -20
I_maxx = 20
error_x = 0
prev_error_x = 0

# Y PID Constants
Kpy = 200.
Kdy = 0.1
Kiy = 0.1
I_y = 0
I_miny = -20
I_maxy = 20
error_y = 0
prev_error_y = 0

its = 50000
counter = 0
sin_in = 0

for k in range(1,its):
	# Calculate for X
	Ux1.append(Ux1[k-1]+(Ux1_dot*ts))
	Ux2.append(Ux2[k-1]+(Ux2_dot*ts))
	Ox1.append(Ux1[k-1]+(Ux1_dot*ts)+Px1)
	Ox2.append(Ux2[k-1]+(Ux2_dot*ts)+Px2)

	# X PID Controller
	error_x = Ox1[k-1] - Ox2[k-1]
	P_x = Kpx*error_x
	D_x = Kdx*(error_x-prev_error_x)
	I_x = I_x + error_x
	if I_x > I_maxx:
		I_x = I_maxx
	elif I_x < I_minx:
		I_x = I_minx
	I_x = Kix*I_x
	prev_error_x = error_x

	Ux2_dot = P_x+D_x+I_x

	if Ux2_dot > max_vel:
		Ux2_dot = max_vel
	elif Ux2_dot < -max_vel:
		Ux2_dot = -max_vel
	

	# Calculate for Y
	Uy1.append(Uy1[k-1]+(Uy1_dot*ts))
	Uy2.append(Uy2[k-1]+(Uy2_dot*ts))
	Oy1.append(Uy1[k-1]+(Uy1_dot*ts)+Py1)
	Oy2.append(Uy2[k-1]+(Uy2_dot*ts)+Py2)

	# Y PID Controller
	error_y = Oy1[k-1] - Oy2[k-1]
	P_y = Kpy*error_y
	D_y = Kdy*(error_y-prev_error_y)
	I_y = I_y + error_y
	if I_y > I_maxx:
		I_y = I_maxx
	elif I_y < I_minx:
		I_y = I_minx
	I_y = Kiy*I_y
	prev_error_y = error_y

	Uy2_dot = P_y+D_y+I_y

	if Uy2_dot > max_vel:
		Uy2_dot = max_vel
	elif Uy2_dot < -max_vel:
		Uy2_dot = -max_vel

	# Calculate for z
	Uz1.append(Uz1[k-1]+(Uz1_dot*ts))
	Uz1_dot = 0.0
	Uz2_dot = 0.0
	Uz2.append(Uz2[k-1]+(Uz2_dot*ts))
	Oz1.append(Uz1[k-1]+(Uz1_dot*ts)+Pz1)
	Oz2.append(Uz2[k-1]+(Uz2_dot*ts)+Pz2)
	count.append(k)
	#print Uy1[k], Uy1_dot
	
				
	if counter == 0:
		Ux1_dot = 0.3
		Uy1_dot = 0.0
	if counter== 8000:
		Ux1_dot = 0.0
		Uy1_dot = 0.3
	if counter == 16000:
		Ux1_dot = 0.3
		Uy1_dot = 0.0
	if counter == 24000:
		Ux1_dot = 0.0
		Uy1_dot = -0.3
	if counter == 32000:
		counter = -1
	counter += 1

	'''Ux1_dot = 0.5*math.sin(sin_in)
	Uy1_dot = 0.5*math.sin(sin_in+3.14/2)
	sin_in += 0.0001
	if sin_in >= 2*3.14:
		sin_in = 0.'''

	with open('/home/ace/catkin_ws/src/Quadcopter_Stereo_Camera_Formation_Control/results/PID/results.csv', 'a') as fp:
				r_writer = csv.writer(fp, delimiter=',')
				r_writer.writerow([Ux1[k],Uy1[k],Ux2[k],Uy2[k]])
	

	

### X Plotting Stuff
remove_amount = 4500
'''plt.figure()
plt.plot(count,Ux1)
plt.plot(count,Ux2)
plt.title("Actual Poses of UAVs (X)")'''
plt.figure()
plt.title("Actual Poses of Points on Ground (X)")
'''# Rise Time X Poses
Rt_index = next(x[0] for x in enumerate(Ox2) if x[1] > 0.999)
Rt = ts*(Rt_index+1)
plt.scatter(Rt_index,Ox2[Rt_index],c='g')
plt.scatter(0,Ox2[Rt_index],c='g')
plt.plot([0, Rt_index],[Ox2[Rt_index],Ox2[Rt_index]],'g')
plt.text(0, Ox2[Rt_index]+.01, 'Rt=Pt=' + str(Rt) + 'secs', fontsize=10, color='g')
# Overshoot X Poses
Os = max(Ox2)
Os_actual = Ox1[0]-Os
plt.text(Rt_index+115, Ox2[Rt_index]+.01, 'Overshoot=' + str(float(Os_actual)), fontsize=10, color='g')

#Delay Time X
Dt_index = next(x[0] for x in enumerate(Ox2) if x[1] > 0.5)
Dt = ts*(Dt_index+1)
plt.scatter(Dt_index,Ox2[Dt_index],c='g')
plt.scatter(0,Ox2[Dt_index],c='g')
plt.plot([0, Dt_index],[Ox2[Dt_index],Ox2[Dt_index]],'g')
plt.text(0, Ox2[Dt_index]+.01, 'Dt=' + str(Dt) + 'secs', fontsize=10, color='g')'''

# Plot X Poses
plt.plot(Ox2,'b')
plt.plot(Ox1,'r--')

# Error Plot
plt.figure()
plt.title("Error of Points on the Ground (X)")
plt.plot([a-b for a,b in zip(Ox1[:-10*remove_amount],Ox2[:-10*remove_amount])])

### Y Plotting Stuff
'''
plt.figure()
plt.plot(count,Uy1)
plt.plot(count,Uy2)
plt.title("Actual Poses of UAVs (Y)")'''
plt.figure()
plt.title("Actual Poses of Points on Ground (Y)")
'''# Rise Time Y Poses
Rt_index = next(x[0] for x in enumerate(Oy2) if x[1] > 0.999)
Rt = ts*(Rt_index+1)
plt.scatter(Rt_index,Oy2[Rt_index],c='g')
plt.scatter(0,Oy2[Rt_index],c='g')
plt.plot([0, Rt_index],[Oy2[Rt_index],Oy2[Rt_index]],'g')
plt.text(0, Oy2[Rt_index]+.01, 'Rt=Pt=' + str(Rt) + 'secs', fontsize=10, color='g')
# Overshoot Y Poses
Os = max(Oy2)
Os_actual = Oy1[0]-Os
plt.text(Rt_index+115, Oy2[Rt_index]+.01, 'Overshoot=' + str(int(Os_actual)), fontsize=10, color='g')

#Delay Time Y
Dt_index = next(x[0] for x in enumerate(Oy2) if x[1] > 0.5)
Dt = ts*(Dt_index+1)
plt.scatter(Dt_index,Oy2[Dt_index],c='g')
plt.scatter(0,Oy2[Dt_index],c='g')
plt.plot([0, Dt_index],[Oy2[Dt_index],Oy2[Dt_index]],'g')
plt.text(0, Oy2[Dt_index]+.01, 'Dt=' + str(Dt) + 'secs', fontsize=10, color='g')'''

# Plot Y Poses
plt.plot(Oy2,'b')
plt.plot(Oy1,'r--')

# Error Plot
plt.figure()
plt.title("Error of Points on the Ground (Y)")
plt.plot([a-b for a,b in zip(Oy1[:-10*remove_amount],Oy2[:-10*remove_amount])])

# Plot Z Poses
plt.figure()
plt.plot(count,Uz1)
plt.plot(count,Uz2)
plt.title("Actual Poses of UAVs (Z)")
plt.figure()
plt.title("Actual Poses of Points on Ground (Z)")
plt.plot(Oz1)
plt.plot(Oz2)

# 3D Plot of UAVs
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(Ux2,Uy2,Uz2, 'b', label='Follower MAV Pose')
ax.plot(Ux1,Uy1,Uz1, 'r', label='Leader MAV Pose')
ax.plot(Ox2,Oy2,Oz2, 'c--', markersize=2, label='Follower Trackpoint Pose')
ax.plot(Ox1,Oy1,Oz1, 'm--', linewidth=6, label='Leader Trackpoint Pose')
ax.legend(prop={'size': 20})

plt.show()
