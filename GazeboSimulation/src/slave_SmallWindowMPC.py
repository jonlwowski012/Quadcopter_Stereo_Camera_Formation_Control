#!/usr/bin/env python

# Title: PID Control for UAV Stereo Camera Formations
# Description: UAV uses PID control to follow a master UAV
# Engineer: Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com
# Lab: Autonomous Controls Lab, The University of Texas at San Antonio


### Import Libraries
import sys
import rospy
from geometry_msgs.msg import Twist, Vector3Stamped
from geometry_msgs.msg import PoseArray
from gazebo_msgs.msg import ModelStates
import math
import numpy as np
import tf
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import StereoCameraModel
from stereo_msgs.msg import DisparityImage
import image_geometry
from sensor_msgs.msg import CameraInfo
import cv2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import csv
import os

### Init Globals ###
uav_location = 0
master_location = 0
posearr = []
master_projection = 0
slave_projection = 0

### Image Processing Class To Calculate Projection Vector 
class image_converter:

	### Initialize Camera Class
	def __init__(self):
		r = rospy.Rate(50) # 1hz
		self.listener = tf.TransformListener()
		self.image_pub = rospy.Publisher("mask_image1",Image,queue_size=10)
		self.location_pub = rospy.Publisher("slave_point",Pose,queue_size=10)

		#cv2.namedWindow("Image window", 1)
		self.bridge = CvBridge()
		self.cam_info_sub = rospy.Subscriber("/uav2/right/camera_info",CameraInfo,self.callbackinforight)
		self.cam_info_sub = rospy.Subscriber("/uav2/left/camera_info",CameraInfo,self.callbackinfoleft)
		self.image_sub = rospy.Subscriber("/uav2/left/image_raw",Image,self.callback)
		self.dispairity_sub = rospy.Subscriber("/uav2/disparity",DisparityImage,self.callbackDisparity)
		self.info_msg_right = CameraInfo()
		self.info_msg_left = CameraInfo()
		self.cam_model = StereoCameraModel()
		self.Disparity = DisparityImage()
		self.disparity = None
		
	### Callback for Disparity from Stereo Camera
	def callbackDisparity(self,data):
		try:
			self.Disparity = data
		except CvBridgeError, e:
			print e
			 
	### Callbacks for images from stereo camera
	def callbackinforight(self,data):
		 self.info_msg_right = data
	def callbackinfoleft(self,data):
		 self.info_msg_left = data

	### Image processing callback
	def callback(self,data):
		### Convert from ROS to OpenCV image
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e

		mask = cv_image
		
		### Convert dispairty to opencv type
		try:
			if self.Disparity.image.encoding != []:
				self.disparity = self.bridge.imgmsg_to_cv2(self.Disparity.image,"passthrough")
		except CvBridgeError, e:
			print e

		### Check if dispairty exists
		if np.any(self.disparity) != None:

			### Calculate track point location
			(rows,cols,channels) = cv_image.shape
			overlap_percent = .1
			track_point = (int(math.floor(rows-(overlap_percent*rows))), int(math.floor(cols/2)))
			cv2.circle(cv_image,track_point, 5, (255,0,0), -1)
		
			### Setup stereo camera model ###
			self.cam_model.fromCameraInfo(self.info_msg_left,self.info_msg_right)
		
			point_msg = PoseStamped()

			global slave_projection
			global uav_location

			try:
				### Project pixel location to 3D ray
				cam_model_point = self.cam_model.projectPixelTo3d((track_point[0],track_point[1]),self.disparity[track_point[0]][track_point[1]])

				### Store 3D ray into Pose Msg
				point_msg.pose.position.x= cam_model_point[0]
				point_msg.pose.position.y=cam_model_point[1]
				point_msg.pose.position.z= cam_model_point[2]
				point_msg.pose.orientation =uav_location.orientation
				point_msg.header.stamp = rospy.Time.now()
				point_msg.header.frame_id = self.cam_model.tfFrame()
				slave_projection = point_msg
			except:
				print "failed"

	
			try: 
				### Use Tf to tranfrom 3D ray into world frame
				self.listener.waitForTransform(self.cam_model.tfFrame(), "uav2/uav2/nav", rospy.Time.now(), rospy.Duration(1))
				tf_point = self.listener.transformPose("uav2/uav2/nav", point_msg)

				### Store 3D ray in world frame into Pose Msg
				pose_msg = Pose()
				pose_msg.position.x = tf_point.pose.position.x
				pose_msg.position.y = tf_point.pose.position.y 
				pose_msg.position.z = tf_point.pose.position.z
				pose_msg.orientation = uav_location.orientation

				flag_add = True
			
				#print tf_point
			except tf.ConnectivityException:
				rospy.logwarn('ConnectivityException')
			except tf.LookupException:
				rospy.logwarn('LookupException')
			except tf.ExtrapolationException:
				rospy.logwarn('ExtrapolationException')
				
			### Publish point on ground location
			slave_point = pose_msg
			self.location_pub.publish(pose_msg)
			
			### End of Image Processing ######
			#cv2.imshow('Slave', cv_image)
		cv2.waitKey(3)



### Subscribe to the EKF Position of the Slave
def uavCallback_ekf(data):
	global uav_location
	try:
		point_msg = PoseStamped()
		point_msg.pose = data.pose
		point_msg.header.stamp = rospy.Time.now()
		point_msg.header.frame_id = "uav2/nav"
		listener.waitForTransform("/world", "uav2/nav", rospy.Time.now(), rospy.Duration(1))
		uav_location_temp = listener.transformPose("/world", point_msg)
		uav_location = uav_location_temp.pose
	except:
		print "failed"
	
def uav_location_sub_ekf():
	rospy.Subscriber("sensor_pose", PoseStamped, uavCallback_ekf) 


### Subscribe to the EKF Position of Master	
def masterCallback_ekf(data):
	global master_location
	try:
		point_msg = PoseStamped()
		point_msg.pose = data.pose
		point_msg.header.stamp = rospy.Time.now()
		point_msg.header.frame_id = "uav1/nav"
		listener.waitForTransform("/world", "uav1/nav", rospy.Time.now(), rospy.Duration(1))
		master_location_temp = listener.transformPose("/world", point_msg)
		master_location = master_location_temp.pose
	except:
		print "failed"

	
def master_location_sub_ekf():
	rospy.Subscriber("/uav1/sensor_pose", PoseStamped, masterCallback_ekf) 


### Subscriber for Master's Velocity
def masterCallback_vel(data):
	global master_velocity
	master_velocity = data

def master_velocity_sub():
	rospy.Subscriber("/uav1/cmd_vel", Twist, masterCallback_vel) 



### Set Velocity of UAV Function
def set_velocity_uav(lx1, ly1, lz1, ax1, ay1, az1):   
    pub1 = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
   
    r = rospy.Rate(10) # 10hz
    command1 = Twist()
    command1.linear.x = lx1
    command1.linear.y = ly1
    command1.linear.z = lz1
    command1.angular.x = ax1
    command1.angular.y = ay1
    command1.angular.z = az1
    hover = Twist()
    hover.linear.x = 0.0
    hover.linear.y = 0.0
    hover.linear.z = 0.0
    hover.angular.x = 0.0
    hover.angular.y = 0.0
    hover.angular.z = 0.0
    pub1.publish(command1)
    

### PID controller function
def mav_controller_PID():
	global master_projection
	global slave_projection
	global uav_location
	global master_location
	global master_velocity
	flag = True
	dt = 0.001
	phi = np.matrix([[0.,0.,1.,0,1,0],[0,0,0,1,0,1],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
	gam = np.matrix([[dt,0],[0.,dt],[dt,0],[0,dt],[0,0],[0,0]])
	window_size = 10
	Q = np.matrix([[10, 0, 0, 0, 0, 0],[0, 10, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0]])
	R = np.matrix([[1, 0],[0,1]])
	index = 0
	k=0

	### Controller Loop
	while not rospy.is_shutdown():
		### Wait for necessary info to be available
		while type(master_projection) is int:
			a = 1
		while type(slave_projection) is int:
			a = 1
		while master_location == 0:
			a = 1
		while uav_location == 0:
			a = 1	

		### Calculate States
		master_states1 = np.matrix([[master_location.position.x+master_projection.pose.position.x],[master_location.position.y+master_projection.pose.position.y],[master_location.position.x],[master_location.position.y],[master_projection.pose.position.x],[master_projection.pose.position.y]])
		slave_states = np.matrix([[uav_location.position.x+slave_projection.pose.position.x],[uav_location.position.y+slave_projection.pose.position.y],[uav_location.position.x],[uav_location.position.y],[slave_projection.pose.position.x],[slave_projection.pose.position.y]])	
	
		# MPC Controller
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
				
		r=np.matrix([[master_states1[0,0]+48],[master_states1[1,0]],[master_states1[2,0]+slave_states[4,0]-master_states1[4,0]+48],[master_states1[3,0]+slave_states[5,0]-master_states1[5,0]],[slave_states[4,0]],[slave_states[5,0]]])
		Us = M[index]*(slave_states-r)

		slave_velocity_x=Us[0,0]
		slave_velocity_y=Us[1,0]

		if slave_velocity_x > 1.0:
			slave_velocity_x = 1.0
		elif slave_velocity_x < -1.0:
			slave_velocity_x = -1.0
		
		if slave_velocity_y > 1.0:
			slave_velocity_y = 1.0
		elif slave_velocity_y < -1.0:
			slave_velocity_y = -1.0
		
		### Save Data to CSV File
		if flag == True:
			try:
	    			os.remove('/home/ace/catkin_ws/src/GazeboSimulation/results/MPC/small_error.csv')
				flag = False
			except OSError:
	    			pass
		with open('/home/ace/catkin_ws/src/GazeboSimulation/results/MPC/small_error.csv', 'a') as fp:
			r_writer = csv.writer(fp, delimiter=',')
			r_writer.writerow([master_location.position.x,uav_location.position.x,master_projection.pose.position.x,slave_projection.pose.position.x,master_location.position.y,uav_location.position.y,master_projection.pose.position.y,slave_projection.pose.position.y])
		#print "Values: ", PD_p, PD_r

		### Set Velocity of Slave
		set_velocity_uav(slave_velocity_x,slave_velocity_y,0,0,0,0)
		k+=1

		time.sleep(dt)

### Function to Take off to a desired height	
def takeoff_height(height):
	global uav_location
	while(uav_location == 0):
		a=1 
	while(uav_location.position.z < height):
		set_velocity_uav(0,0,1.0,0,0,0)
	set_velocity_uav(0.0,0,0,0,0,0)

### Subscriber for Master's Projection Vector
def masterprojectionCallback(data):
	global master_projection
	master_projection = data

def master_projection_sub():
	rospy.Subscriber("/uav1/master_projection", PoseStamped, masterprojectionCallback)


### Main Function 	
if __name__ == '__main__':
	global pose_array

	### Define TF Listener ###
	listener = tf.TransformListener()

	### Rospy Init ####
	rospy.init_node('slave', anonymous=True)

	### Subscribe to UAV Info ###
	uav_location_sub_ekf()
	master_location_sub_ekf()
	master_projection_sub()
	master_velocity_sub()

	### Sleep to give world time to build ###
	rospy.sleep(4.)

 	### Take off to given height ###
	takeoff_height(5)
	
	### Begin Image Processing ###
	image_converter()

	### Sleep to give world time to build ###
	rospy.sleep(2.)

	try:
		### Have Mav fly across the world ###
		mav_controller_PID()
			
	



	except rospy.ROSInterruptException: pass
