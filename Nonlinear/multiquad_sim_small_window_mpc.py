import numpy as np
import math
import scipy.integrate
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import time
import datetime
import threading
import signal
import sys
import csv
import random

ctrl1=None
ctrl2=None
ctrl3=None
quad=None
	
# Constants
TIME_SCALING = 0.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
QUAD_DYNAMICS_UPDATE = 0.005 # seconds
CONTROLLER_DYNAMICS_UPDATE = 0.005 # seconds

# Define the quads
quads={'q1':{'position':[0,0,2],'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10,4.5],'weight':1.2,'mprj':[1,0,0],'sprj':[-1,0,0]},
	'q2':{'position':[10,10,2],'orientation':[0,0,0],'L':0.15,'r':0.05,'prop_size':[6,4.5],'weight':0.7,'mprj':[1,0,0],'sprj':[-1,0,0]},
	'q3':{'position':[-10,10,2],'orientation':[0,0,0],'L':0.15,'r':0.05,'prop_size':[6,4.5],'weight':0.7,'mprj':[1,0,0],'sprj':[-1,0,0]}}
# Controller parameters
CONTROLLER_1_PARAMETERS = {'Motor_limits':[4000,9000],
					'Tilt_limits':[-10,10],
					'Z_XY_offset':500,
					'Linear_PID':{'P':[2000,2000,7000],'I':[0.25,0.25,4.5],'D':[50,50,5000]},
					'Linear_To_Angular_Scaler':[0.01,1,0],
					'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,0.01],'D':[12000,12000,1800]},
					}
CONTROLLER_2_PARAMETERS = {'Motor_limits':[4000,9000],
					'Tilt_limits':[-10,10],
					'Z_XY_offset':500,
					'Linear_PID':{'P':[2000,2000,7000],'I':[0.25,0.25,4.5],'D':[50,50,5000]},
					'Linear_To_Angular_Scaler':[1,1,0],
					'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,0.01],'D':[12000,12000,1800]},
					}
					
CONTROLLER_3_PARAMETERS = {'Motor_limits':[4000,9000],
					'Tilt_limits':[-10,10],
					'Z_XY_offset':500,
					'Linear_PID':{'P':[2000,2000,7000],'I':[0.25,0.25,4.5],'D':[50,50,5000]},
					'Linear_To_Angular_Scaler':[1,1,0],
					'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,0.01],'D':[12000,12000,1800]},
					}
# Set goals to go to
GOALS_1 = [(0,0,2),(1,0,2),(0,1,2),(-1,0,2),(0,-1,2)]
GOALS_2 = [(0,0,4),(-1,0,4),(0,-1,4),(1,0,4),(0,1,4)]

# Global variable
run = True

class Propeller():
	def __init__(self, prop_dia, prop_pitch, thrust_unit='N'):
		self.dia = prop_dia
		self.pitch = prop_pitch
		self.thrust_unit = thrust_unit
		self.speed = 0 #RPM
		self.thrust = 0

	def set_speed(self,speed):
		self.speed = speed
		# From http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
		self.thrust = 4.392e-8 * self.speed * math.pow(self.dia,3.5)/(math.sqrt(self.pitch))
		self.thrust = self.thrust*(4.23e-4 * self.speed * self.pitch)
		if self.thrust_unit == 'Kg':
			self.thrust = self.thrust*0.101972

class Quadcopter():
	# State space representation: [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
	# From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
	def __init__(self,quads,gravity=9.81,b=0.0245):
		self.quads = quads
		self.g = gravity
		self.b = b
		self.thread_object = None
		self.ode =  scipy.integrate.ode(self.state_dot).set_integrator('vode',nsteps=500,method='bdf')
		self.time = datetime.datetime.now()
		for key in self.quads:
			self.quads[key]['state'] = np.zeros(12)
			self.quads[key]['state'][0:3] = self.quads[key]['position']
			self.quads[key]['state'][6:9] = self.quads[key]['orientation']
			self.quads[key]['m1'] = Propeller(self.quads[key]['prop_size'][0],self.quads[key]['prop_size'][1])
			self.quads[key]['m2'] = Propeller(self.quads[key]['prop_size'][0],self.quads[key]['prop_size'][1])
			self.quads[key]['m3'] = Propeller(self.quads[key]['prop_size'][0],self.quads[key]['prop_size'][1])
			self.quads[key]['m4'] = Propeller(self.quads[key]['prop_size'][0],self.quads[key]['prop_size'][1])
			# From Quadrotor Dynamics and Control by Randal Beard
			ixx=((2*self.quads[key]['weight']*self.quads[key]['r']**2)/5)+(2*self.quads[key]['weight']*self.quads[key]['L']**2)
			iyy=ixx
			izz=((2*self.quads[key]['weight']*self.quads[key]['r']**2)/5)+(4*self.quads[key]['weight']*self.quads[key]['L']**2)
			self.quads[key]['I'] = np.array([[ixx,0,0],[0,iyy,0],[0,0,izz]])
			self.quads[key]['invI'] = np.linalg.inv(self.quads[key]['I'])

	def rotation_matrix(self,angles):
		ct = math.cos(angles[0])
		cp = math.cos(angles[1])
		cg = math.cos(angles[2])
		st = math.sin(angles[0])
		sp = math.sin(angles[1])
		sg = math.sin(angles[2])
		R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
		R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
		R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
		R = np.dot(R_z, np.dot( R_y, R_x ))
		return R

	def state_dot(self, time, state, key):
		state_dot = np.zeros(12)
		# The velocities(t+1 x_dots equal the t x_dots)
		state_dot[0] = self.quads[key]['state'][3]
		state_dot[1] = self.quads[key]['state'][4]
		state_dot[2] = self.quads[key]['state'][5]
		# The acceleration
		x_dotdot = np.array([0,0,-self.quads[key]['weight']*self.g]) + np.dot(self.rotation_matrix(self.quads[key]['state'][6:9]),np.array([0,0,(self.quads[key]['m1'].thrust + self.quads[key]['m2'].thrust + self.quads[key]['m3'].thrust + self.quads[key]['m4'].thrust)]))/self.quads[key]['weight']
		state_dot[3] = x_dotdot[0]
		state_dot[4] = x_dotdot[1]
		state_dot[5] = x_dotdot[2]
		# The angular rates(t+1 theta_dots equal the t theta_dots)
		state_dot[6] = self.quads[key]['state'][9]
		state_dot[7] = self.quads[key]['state'][10]
		state_dot[8] = self.quads[key]['state'][11]
		# The angular accelerations
		omega = self.quads[key]['state'][9:12]
		tau = np.array([self.quads[key]['L']*(self.quads[key]['m1'].thrust-self.quads[key]['m3'].thrust), self.quads[key]['L']*(self.quads[key]['m2'].thrust-self.quads[key]['m4'].thrust), self.b*(self.quads[key]['m1'].thrust-self.quads[key]['m2'].thrust+self.quads[key]['m3'].thrust-self.quads[key]['m4'].thrust)])
		omega_dot = np.dot(self.quads[key]['invI'], (tau - np.cross(omega, np.dot(self.quads[key]['I'],omega))))
		state_dot[9] = omega_dot[0]
		state_dot[10] = omega_dot[1]
		state_dot[11] = omega_dot[2]
		return state_dot

	def update(self, dt):
		for key in self.quads:
			self.ode.set_initial_value(self.quads[key]['state'],0).set_f_params(key)
			self.quads[key]['state'] = self.ode.integrate(self.ode.t + dt)
			self.quads[key]['state'][2] = max(0,self.quads[key]['state'][2])

	def set_motor_speeds(self,quad_name,speeds):
		self.quads[quad_name]['m1'].set_speed(speeds[0])
		self.quads[quad_name]['m2'].set_speed(speeds[1])
		self.quads[quad_name]['m3'].set_speed(speeds[2])
		self.quads[quad_name]['m4'].set_speed(speeds[3])
		
	def get_projection_calculation(self,quad_name,slave):
		# Projections
		if slave == True:
			q1_pos = self.get_position(quad_name)
			q1_projection = [self.quads[quad_name]['sprj'][0],self.quads[quad_name]['sprj'][1],-q1_pos[2]]
		elif slave == False:
			q1_pos = self.get_position(quad_name)
			q1_projection = [self.quads[quad_name]['mprj'][0],self.quads[quad_name]['mprj'][1],-q1_pos[2]]
			
		self.quads[quad_name]['sprj'][2] = -q1_pos[2]
		self.quads[quad_name]['mprj'][2] = -q1_pos[2]
		return q1_projection
		
	def get_position(self,quad_name):
		return self.quads[quad_name]['state'][0:3]

	def get_linear_rate(self,quad_name):
		return self.quads[quad_name]['state'][3:6]

	def get_orientation(self,quad_name):
		return self.quads[quad_name]['state'][6:9]

	def get_angular_rate(self,quad_name):
		return self.quads[quad_name]['state'][9:12]

	def get_state(self,quad_name):
		return self.quads[quad_name]['state']

	def set_position(self,quad_name,position):
		self.quads[quad_name]['state'][0:3] = position

	def set_orientation(self,quad_name,orientation):
		self.quads[quad_name]['state'][6:9] = orientation
		

	def get_time(self):
		return self.time

	def thread_run(self,dt,time_scaling):
		rate = time_scaling*dt
		last_update = self.time
		while(run==True):
			time.sleep(0)
			self.time = datetime.datetime.now()
			if (self.time-last_update).total_seconds() > rate:
				self.update(dt)
				last_update = self.time

	def start_thread(self,dt=0.002,time_scaling=1):
		self.thread_object = threading.Thread(target=self.thread_run,args=(dt,time_scaling))
		self.thread_object.start()

class GUI():
	# 'quad_list' is a dictionary of format: quad_list = {'quad_1_name':{'position':quad_1_position,'orientation':quad_1_orientation,'arm_span':quad_1_arm_span}, ...}
	def __init__(self, quads):
		self.quads = quads
		self.fig = plt.figure()
		self.ax = Axes3D.Axes3D(self.fig)
		self.ax.set_xlim3d([0., 6.0])
		self.ax.set_xlabel('X')
		self.ax.set_ylim3d([0., 6.0])
		self.ax.set_ylabel('Y')
		self.ax.set_zlim3d([0, 5.0])
		self.ax.set_zlabel('Z')
		self.ax.set_title('Quadcopter Simulation')
		self.init_plot()

	def rotation_matrix(self,angles):
		ct = math.cos(angles[0])
		cp = math.cos(angles[1])
		cg = math.cos(angles[2])
		st = math.sin(angles[0])
		sp = math.sin(angles[1])
		sg = math.sin(angles[2])
		R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
		R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
		R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
		R = np.dot(R_z, np.dot( R_y, R_x ))
		return R

	def init_plot(self):
		for key in self.quads:
			self.quads[key]['l1'], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
			self.quads[key]['l2'], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
			self.quads[key]['hub'], = self.ax.plot([],[],[],marker='o',color='green', markersize=6,antialiased=False)
			self.quads[key]['mdprj'], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
			self.quads[key]['sdprj'], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
			self.quads[key]['camprj'], = self.ax.plot([],[],[],color='green',linewidth=3,antialiased=False)
			self.quads[key]['pose_history_x']=[]
			self.quads[key]['pose_history_y']=[]
			self.quads[key]['pose_history_z']=[]
			index = int(key.replace("q",""))-1
			colors = ['0.5','0.7','0.7']
			self.quads[key]['pos_line'], = self.ax.plot([],[],[],color=colors[index],linewidth=0.1,antialiased=False)

	def update(self):
		for key in self.quads:
			R = self.rotation_matrix(self.quads[key]['orientation'])
			L = self.quads[key]['L']
			points = np.array([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T
			points = np.dot(R,points)
			points[0,:] += self.quads[key]['position'][0]
			points[1,:] += self.quads[key]['position'][1]
			points[2,:] += self.quads[key]['position'][2]
			
			self.quads[key]['pose_history_x'].append(self.quads[key]['position'][0])
			self.quads[key]['pose_history_y'].append(self.quads[key]['position'][1])
			self.quads[key]['pose_history_z'].append(self.quads[key]['position'][2])
			self.quads[key]['pos_line'].set_data(self.quads[key]['pose_history_x'],self.quads[key]['pose_history_y'])
			self.quads[key]['pos_line'].set_3d_properties(np.zeros((len(self.quads[key]['pose_history_x']))))
			
			self.quads[key]['l1'].set_data(points[0,0:2],points[1,0:2])
			self.quads[key]['l1'].set_3d_properties(points[2,0:2])
			self.quads[key]['l2'].set_data(points[0,2:4],points[1,2:4])
			self.quads[key]['l2'].set_3d_properties(points[2,2:4])
			self.quads[key]['hub'].set_data(points[0,5],points[1,5])
			self.quads[key]['hub'].set_3d_properties(points[2,5])
			self.quads[key]['mdprj'].set_data([self.quads[key]['position'][0],self.quads[key]['position'][0]+self.quads[key]['mprj'][0]],[self.quads[key]['position'][1],self.quads[key]['position'][1]+self.quads[key]['mprj'][1]])
			self.quads[key]['mdprj'].set_3d_properties([self.quads[key]['position'][2],self.quads[key]['position'][2]+self.quads[key]['mprj'][2]])
			self.quads[key]['sdprj'].set_data([self.quads[key]['position'][0],self.quads[key]['position'][0]+self.quads[key]['sprj'][0]],[self.quads[key]['position'][1],self.quads[key]['position'][1]+self.quads[key]['sprj'][1]])
			self.quads[key]['sdprj'].set_3d_properties([self.quads[key]['position'][2],self.quads[key]['position'][2]+self.quads[key]['sprj'][2]])
			cam_size = 1.5
			self.quads[key]['camprj'].set_data([self.quads[key]['position'][0]-cam_size,self.quads[key]['position'][0]+cam_size,self.quads[key]['position'][0]+cam_size,self.quads[key]['position'][0]-cam_size,self.quads[key]['position'][0]-cam_size],[self.quads[key]['position'][1]-cam_size,self.quads[key]['position'][1]-cam_size,self.quads[key]['position'][1]+cam_size,self.quads[key]['position'][1]+cam_size,self.quads[key]['position'][1]-cam_size])
			self.quads[key]['camprj'].set_3d_properties([0.0,0.0,0.0,0.0,0.0])
		plt.pause(0.000000000000001)

class Controller():
	def __init__(self, quad_identifier, get_state, get_time, actuate_motors, params):
		self.quad_identifier = quad_identifier
		self.actuate_motors = actuate_motors
		self.get_state = get_state
		self.get_time = get_time
		self.MOTOR_LIMITS = params['Motor_limits']
		self.TILT_LIMITS = [(params['Tilt_limits'][0]/180.0)*3.14,(params['Tilt_limits'][1]/180.0)*3.14]
		self.Z_LIMITS = [self.MOTOR_LIMITS[0]+params['Z_XY_offset'],self.MOTOR_LIMITS[1]-params['Z_XY_offset']]
		self.LINEAR_P = params['Linear_PID']['P']
		self.LINEAR_I = params['Linear_PID']['I']
		self.LINEAR_D = params['Linear_PID']['D']
		self.LINEAR_TO_ANGULAR_SCALER = params['Linear_To_Angular_Scaler']
		self.ANGULAR_P = params['Angular_PID']['P']
		self.ANGULAR_I = params['Angular_PID']['I']
		self.ANGULAR_D = params['Angular_PID']['D']
		self.xi_term = 0
		self.yi_term = 0
		self.zi_term = 0
		self.thetai_term = 0
		self.phii_term = 0
		self.gammai_term = 0
		self.thread_object = None
		self.target = [0,0,0]

	def constrain_list(self,quantities,c_min,c_max):
		for i in range(len(quantities)):
			quantities[i] = min(max(quantities[i],c_min),c_max)

	def constrain(self,quantity,c_min,c_max):
		return min(max(quantity,c_min),c_max)

	def update(self):
		[dest_x,dest_y,dest_z] = self.target
		[x,y,z,x_dot,y_dot,z_dot,theta,phi,gamma,theta_dot,phi_dot,gamma_dot] = self.get_state(self.quad_identifier)
		x_error = dest_x-x_dot
		y_error = dest_y-y_dot
		z_error = dest_z-z
		self.xi_term += self.LINEAR_I[0]*x_error
		self.yi_term += self.LINEAR_I[1]*y_error
		self.zi_term += self.LINEAR_I[2]*z_error
		dest_x_dot = self.LINEAR_P[0]*(x_error) + self.LINEAR_D[0]*(-x_dot) + self.xi_term
		dest_y_dot = self.LINEAR_P[1]*(y_error) + self.LINEAR_D[1]*(-y_dot) + self.yi_term
		dest_z_dot = self.LINEAR_P[2]*(z_error) + self.LINEAR_D[2]*(-z_dot) + self.zi_term
		throttle = self.constrain(dest_z_dot,self.Z_LIMITS[0],self.Z_LIMITS[1])
		dest_theta = self.LINEAR_TO_ANGULAR_SCALER[0]*(dest_x_dot*math.sin(gamma)-dest_y_dot*math.cos(gamma))
		dest_phi = self.LINEAR_TO_ANGULAR_SCALER[1]*(dest_x_dot*math.cos(gamma)+dest_y_dot*math.sin(gamma))
		dest_gamma = 0
		dest_theta,dest_phi,dest_gamma = self.constrain(dest_theta,self.TILT_LIMITS[0],self.TILT_LIMITS[1]),self.constrain(dest_phi,self.TILT_LIMITS[0],self.TILT_LIMITS[1]),self.constrain(dest_gamma,self.TILT_LIMITS[0],self.TILT_LIMITS[1])
		theta_error = dest_theta-theta
		phi_error = dest_phi-phi
		gamma_error = dest_gamma-gamma
		self.thetai_term += self.ANGULAR_I[0]*theta_error
		self.phii_term += self.ANGULAR_I[1]*phi_error
		self.gammai_term += self.ANGULAR_I[2]*gamma_error
		x_val = self.ANGULAR_P[0]*(theta_error) + self.ANGULAR_D[0]*(-theta_dot) + self.thetai_term
		y_val = self.ANGULAR_P[1]*(phi_error) + self.ANGULAR_D[1]*(-phi_dot) + self.phii_term
		z_val = self.ANGULAR_P[2]*(gamma_error) + self.ANGULAR_D[2]*(-gamma_dot) + self.gammai_term
		m1 = throttle + x_val + z_val
		m2 = throttle + y_val - z_val
		m3 = throttle - x_val + z_val
		m4 = throttle - y_val - z_val
		M = [m1,m2,m3,m4]
		self.constrain_list(M,self.MOTOR_LIMITS[0],self.MOTOR_LIMITS[1])
		self.actuate_motors(self.quad_identifier,M)

	def update_target(self,target):
		self.target = target

	def thread_run(self,update_rate,time_scaling):
		update_rate = update_rate*time_scaling
		last_update = self.get_time()
		while(run==True):
			time.sleep(0)
			self.time = self.get_time()
			if (self.time - last_update).total_seconds() > update_rate:
				self.update()
				last_update = self.time

	def start_thread(self,update_rate=0.005,time_scaling=1):
		self.thread_object = threading.Thread(target=self.thread_run,args=(update_rate,time_scaling))
		self.thread_object.start()

def signal_handler(signal, frame):
	global run
	run = False
	print('Stopping')
	sys.exit(0)


def form_control():
	global ctrl1
	global ctrl2
	global ctrl3
	global quad
	k=0
	dt = .001
	counter = 0
	phi = np.matrix([[0.,0.,1.,0,1,0],[0,0,0,1,0,1],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
	gam = np.matrix([[dt,0],[0.,dt],[dt,0],[0,dt],[0,0],[0,0]])
	window_size = 10
	Q = np.matrix([[2, 0, 0, 0, 0, 0],[0, 2, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0]])
	R = np.matrix([[.02, 0],[0,.02]])
	index = 0
	
	### Calculate K States
	master_states1 = np.matrix([[quad.get_position('q1')[0]+quad.get_projection_calculation('q1',True)[0]],[quad.get_position('q1')[1]+quad.get_projection_calculation('q1',True)[1]],[quad.get_position('q1')[0]],[quad.get_position('q1')[1]],[quad.get_projection_calculation('q1',True)[0]],[quad.get_projection_calculation('q1',True)[1]]])
	master_states2 = np.matrix([[quad.get_position('q1')[0]+quad.get_projection_calculation('q1',False)[0]],[quad.get_position('q1')[1]+quad.get_projection_calculation('q1',False)[1]],[quad.get_position('q1')[0]],[quad.get_position('q1')[1]],[quad.get_projection_calculation('q1',False)[0]],[quad.get_projection_calculation('q1',False)[1]]])
	slave_states = np.matrix([[quad.get_position('q2')[0]+quad.get_projection_calculation('q2',False)[0]],[quad.get_position('q2')[1]+quad.get_projection_calculation('q2',False)[1]],[quad.get_position('q2')[0]],[quad.get_position('q2')[1]],[quad.get_projection_calculation('q2',False)[0]],[quad.get_projection_calculation('q2',False)[1]]])
	slave2_states = np.matrix([[quad.get_position('q3')[0]+quad.get_projection_calculation('q3',True)[0]],[quad.get_position('q3')[1]+quad.get_projection_calculation('q3',True)[1]],[quad.get_position('q3')[0]],[quad.get_position('q3')[1]],[quad.get_projection_calculation('q3',True)[0]],[quad.get_projection_calculation('q2',True)[1]]])
	time.sleep(5)
	sin_in = 0.	
	while(run==True):
		### Move Master
		t0 = time.time()
		master_vel_x = 0.5*math.sin(sin_in)
		master_vel_y = 0.5*math.sin(sin_in+3.14/2)
		sin_in += 0.0001
		if sin_in >= 2*3.14:
			sin_in = 0.
		'''if counter < 24000:
			master_vel_x = 0.1
			master_vel_y = 0
		elif counter < 48000:
			master_vel_x = 0
			master_vel_y = 0.1
		elif counter < 72000:
			master_vel_x = 0.1
			master_vel_y = 0
		elif counter < 96000:
			master_vel_x = 0
			master_vel_y = -0.1
		else:
			counter = 0'''
		counter += 1
		print counter
		'''if counter == 0:
			master_vel_x = random.uniform(-0.2,0.2)
			master_vel_y = random.uniform(-0.2,0.2)
			counter += 1
		elif counter > 40000:
			counter = 0
		else:
			counter += 1'''
		

		#ctrl1.update_target([master_vel_x,master_vel_y,2])

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
		print Us
		ctrl2.update_target([slave_vel_x,slave_vel_y,2])
		
		#### Slave 2 Controller	
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
				
		r=np.matrix([[master_states2[0,0]],[master_states2[1,0]],[master_states2[2,0]+slave2_states[4,0]-master_states2[4,0]],[master_states2[3,0]+slave2_states[5,0]-master_states2[5,0]],[slave2_states[4,0]],[slave2_states[5,0]]])
		Us = M[index]*(slave2_states-r)
		slave_vel_x=Us[0,0]
		slave_vel_y=Us[1,0]
	
		ctrl3.update_target([slave_vel_x,slave_vel_y,2])

		#### Calculate K States
		master_states1 = np.matrix([[quad.get_position('q1')[0]+quad.get_projection_calculation('q1',True)[0]],[quad.get_position('q1')[1]+quad.get_projection_calculation('q1',True)[1]],[quad.get_position('q1')[0]],[quad.get_position('q1')[1]],[quad.get_projection_calculation('q1',True)[0]],[quad.get_projection_calculation('q1',True)[1]]])
		master_states2 = np.matrix([[quad.get_position('q1')[0]+quad.get_projection_calculation('q1',False)[0]],[quad.get_position('q1')[1]+quad.get_projection_calculation('q1',False)[1]],[quad.get_position('q1')[0]],[quad.get_position('q1')[1]],[quad.get_projection_calculation('q1',False)[0]],[quad.get_projection_calculation('q1',False)[1]]])
		slave_states = np.matrix([[quad.get_position('q2')[0]+quad.get_projection_calculation('q2',False)[0]],[quad.get_position('q2')[1]+quad.get_projection_calculation('q2',False)[1]],[quad.get_position('q2')[0]],[quad.get_position('q2')[1]],[quad.get_projection_calculation('q2',False)[0]],[quad.get_projection_calculation('q2',False)[1]]])
		slave2_states = np.matrix([[quad.get_position('q3')[0]+quad.get_projection_calculation('q3',True)[0]],[quad.get_position('q3')[1]+quad.get_projection_calculation('q3',True)[1]],[quad.get_position('q3')[0]],[quad.get_position('q3')[1]],[quad.get_projection_calculation('q3',True)[0]],[quad.get_projection_calculation('q2',True)[1]]])
		
		index = index + 1;
		if index > window_size-2:
			index = 1
		
		time.sleep(.001)
		dt = time.time()-t0
		k+=1
	
def Run():
	global ctrl1
	global ctrl2
	global ctrl3
	global quad
	
	# Catch Ctrl+C to stop threads
	signal.signal(signal.SIGINT, signal_handler)
	# Make objects for quadcopter, gui and controllers
	gui = GUI(quads=quads)
	quad = Quadcopter(quads=quads)
	ctrl1 = Controller('q1',quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_1_PARAMETERS)
	ctrl2 = Controller('q2',quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_2_PARAMETERS)
	ctrl3 = Controller('q3',quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_3_PARAMETERS)
	# Start the threads
	quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
	ctrl1.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
	ctrl2.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
	ctrl3.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
	
	ctrl1.update_target([0,0,2])
	#ctrl2.update_target([0,0,2])
	#ctrl3.update_target([0,0,2])
	time.sleep(1)
	
	thread_object = threading.Thread(target=form_control)
	thread_object.start()
	
	while(run==True):

		#print slave_vel_x,slave_vel_y
		for key in quads:
			gui.quads[key]['position'] = quad.get_position(key)
			gui.quads[key]['orientation'] = quad.get_orientation(key)
		with open('/home/ace/catkin_ws/src/Quadcopter_Stereo_Camera_Formation_Control/results/MPC/results.csv', 'a') as fp:
			r_writer = csv.writer(fp, delimiter=',')
			r_writer.writerow([quad.get_position('q1')[0],quad.get_position('q1')[1],quad.get_position('q2')[0],quad.get_position('q2')[1],quad.get_position('q3')[0],quad.get_position('q3')[1]])
		gui.update()
		
 
if __name__ == "__main__":
	Run()
