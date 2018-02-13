import math
import numpy as np
import csv
import matplotlib.pyplot as plt
from scipy.interpolate import spline
import scipy.fftpack


remove_amt = 1
with open('/home/ace/catkin_ws/src/GazeboSimulation/results/SFB/error.csv', 'r') as fp:
	reader = csv.reader(fp)
	master_x = []
	slave_x = []
	master_point_x = []
	slave_point_x = []
	master_y = []
	slave_y = []
	master_point_y = []
	slave_point_y = []
	count = []
	i = 0
	for row in reader:
		i+= 1
		count.append(i)
		master_x.append(float(row[0]))
		slave_x.append(float(row[1]))
		master_point_x.append(float(row[2]))
		slave_point_x.append(float(row[3]))
		master_y.append(float(row[4]))
		slave_y.append(float(row[5]))
		master_point_y.append(float(row[6]))
		slave_point_y.append(float(row[7]))



#plt.plot(count, np.poly1d(np.polyfit(count, error_r, 10))(count))
plt.figure()
plt.title("Position Vs Time (EKF X)")
plt.plot(count[:-remove_amt], master_x[:-remove_amt], color='red', label='Master')
plt.plot(count[:-remove_amt], slave_x[:-remove_amt], color='blue', label='Slave')
plt.legend()

plt.figure()
plt.title("Position Vs Time (Track Point X)")
plt.plot(count[:-remove_amt], [a_i + b_i for a_i, b_i in zip(master_point_x, master_x)][:-remove_amt], color='red', label='Master')
plt.plot(count[:-remove_amt], [a_i + b_i for a_i, b_i in zip(slave_point_x, slave_x)][:-remove_amt], color='blue', label='Slave')
plt.legend()

plt.figure()
plt.title("Error Vs Time (Track Point X)")
plt.plot(count[:-remove_amt], [(a+b)-(c+d) for a,b,c,d in zip(master_point_x, master_x,slave_point_x, slave_x)][:-remove_amt], color='red', label='Error')
plt.legend()

plt.figure()
plt.title("Position Vs Time (EKF Y)")
plt.plot(count[:-remove_amt], master_y[:-remove_amt], color='red', label='Master')
plt.plot(count[:-remove_amt], slave_y[:-remove_amt], color='blue', label='Slave')
plt.legend()

plt.figure()
plt.title("Position Vs Time (Track Point Y)")
plt.plot(count[:-remove_amt], [a_i + b_i for a_i, b_i in zip(master_point_y, master_y)][:-remove_amt], color='red', label='Master')
plt.plot(count[:-remove_amt], [a_i + b_i for a_i, b_i in zip(slave_point_y, slave_y)][:-remove_amt], color='blue', label='Slave')
plt.legend()

plt.figure()
plt.title("Error Vs Time (Track Point Y)")
plt.plot(count[:-remove_amt], [(a+b)-(c+d) for a,b,c,d in zip(master_point_y, master_y,slave_point_y, slave_y)][:-remove_amt], color='red', label='Error')
plt.legend()


plt.figure()
plt.title("Actual Position Vs Time (Track Point)")
plt.plot([a_i + b_i for a_i, b_i in zip(master_point_x, master_x)][:-remove_amt], [a_i + b_i for a_i, b_i in zip(master_point_y, master_y)][:-remove_amt], color='red', label='Master')
plt.plot([a_i + b_i for a_i, b_i in zip(slave_point_x, slave_x)][:-remove_amt], [a_i + b_i for a_i, b_i in zip(slave_point_y, slave_y)][:-remove_amt], color='blue', label='Slave')

plt.figure()
plt.title("Actual Position Vs Time (EKF)")
plt.plot(master_x[:-remove_amt], master_y[:-remove_amt], color='red', label='Master')
plt.plot(slave_x[:-remove_amt], slave_y[:-remove_amt], color='blue', label='Slave')
plt.show()
