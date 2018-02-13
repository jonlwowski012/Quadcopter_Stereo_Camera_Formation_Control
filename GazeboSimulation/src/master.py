#!/usr/bin/env python

# Title: State Feedback Control for UAV Stereo Camera Formations
# Description: Master UAV flys around and sends info to slave UAV
# Engineer: Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com
# Lab: Autonomous Controls Lab, The University of Texas at San Antonio


### Import Libraries
import sys
import rospy
from geometry_msgs.msg import Twist 
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

### Init Globals ###
uav_location = 0

### Image Processing Class ###
class image_converter:

	def __init__(self):
		r = rospy.Rate(50) # 1hz
		self.listener = tf.TransformListener()
		self.image_pub = rospy.Publisher("mask_image1",Image,queue_size=10)
		self.location_pub = rospy.Publisher("master_point",Pose,queue_size=10)
		self.projection_pub = rospy.Publisher("master_projection",PoseStamped,queue_size=10)
		#cv2.namedWindow("Image window", 1)
		self.bridge = CvBridge()
		self.cam_info_sub = rospy.Subscriber("right/camera_info",CameraInfo,self.callbackinforight)
		self.cam_info_sub = rospy.Subscriber("left/camera_info",CameraInfo,self.callbackinfoleft)
		self.image_sub = rospy.Subscriber("left/image_raw",Image,self.callback)
		self.dispairity_sub = rospy.Subscriber("disparity",DisparityImage,self.callbackDisparity)
		self.info_msg_right = CameraInfo()
		self.info_msg_left = CameraInfo()
		self.cam_model = StereoCameraModel()
		self.Disparity = DisparityImage()
		self.disparity = None

		
		
	def callbackDisparity(self,data):
		try:
			self.Disparity = data
		except CvBridgeError, e:
			print e
			 
		
	def callbackinforight(self,data):
		 self.info_msg_right = data
	def callbackinfoleft(self,data):
		 self.info_msg_left = data

	def callback(self,data):
		print "entered callback"
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e

		mask = cv_image

		try:
			if self.Disparity.image.encoding != []:
				self.disparity = self.bridge.imgmsg_to_cv2(self.Disparity.image,"passthrough")
		except CvBridgeError, e:
			print e
		if np.any(self.disparity) != None:
			(rows,cols,channels) = cv_image.shape
			overlap_percent = .4
			#track_point = (int(math.floor(rows-(overlap_percent*rows))), int(math.floor(cols/2)))
			track_point = (int(math.floor(0+(overlap_percent*rows))), int(math.floor(cols/2)))
			print "Master: ", track_point
			cv2.circle(cv_image,track_point, 5, (255,0,0), -1)
			### Setup stereo camera model ###

			self.cam_model.fromCameraInfo(self.info_msg_left,self.info_msg_right)

			point_msg = PoseStamped()

			### Coordinate Transformation track point
			global uav_location

			#print centres[i]	
			#print "done"
			try:
				### Project pixel location to 3D ray
				cam_model_point = self.cam_model.projectPixelTo3d((track_point[0],track_point[1]),self.disparity[track_point[0]][track_point[1]])

				### Store 3D ray into Pose Msg
				point_msg.pose.position.x= cam_model_point[0]#*(uav_location.position.z/cam_model_point[2])
				point_msg.pose.position.y=cam_model_point[1]#*(uav_location.position.z/cam_model_point[2])
				point_msg.pose.position.z= cam_model_point[2]#*(uav_location.position.z/cam_model_point[2])
				point_msg.pose.orientation = uav_location.orientation
				point_msg.header.stamp = rospy.Time.now()
				point_msg.header.frame_id = self.cam_model.tfFrame()
				self.projection_pub.publish(point_msg)
			except:
				print "failed"

		
			try: 
				#print self.cam_model.tfFrame()

				### Use Tf to tranfrom 3D ray into world frame

				self.listener.waitForTransform(self.cam_model.tfFrame(), "uav1/uav1/nav", rospy.Time.now(), rospy.Duration(1))
				tf_point = self.listener.transformPose("uav1/uav1/nav", point_msg)

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
			print "master: ", pose_msg
			self.location_pub.publish(pose_msg)
			
			### End of Image Processing ######
		#cv2.imshow('Master', cv_image)
		cv2.waitKey(3)
       


### Subscribe to the EKF Position of Master	
def uavCallback_ekf(data):
	global uav_location
	uav_location = data.pose
	
def uav_location_sub_ekf():
	rospy.Subscriber("sensor_pose", PoseStamped, uavCallback_ekf) 

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
    

### Master UAV Controller   
def mav_controller():
	sin_in = 0.
	while not rospy.is_shutdown():
		count = 0
		while count < 1000000000000000000000:
			count += 1
			master_vel_x = 0.5*math.sin(sin_in)
			master_vel_y = 0.5*math.sin(sin_in+3.14/2)
			sin_in += 0.0001
			if sin_in >= 2*3.14:
				sin_in = 0.
			set_velocity_uav(master_vel_x,master_vel_y,0,0,0,0)
		while count < 200000:
			count += 1
			set_velocity_uav(0.2,0,0,0,0,0)
		while count < 201000:
			count += 1
			set_velocity_uav(0.1,0,0,0,0,0)
		while count < 202000:
			count += 1
			set_velocity_uav(0.0,0,0,0,0,0)
		while count < 203000:
			count += 1
			set_velocity_uav(0,0.1,0,0,0,0)
		while count < 403000:
			count += 1
			set_velocity_uav(0, 0.2,0,0,0,0)
		while count < 404000:
			count += 1
			set_velocity_uav(0,0.1,0,0,0,0)
		while count < 405000:
			count += 1
			set_velocity_uav(0,0,0,0,0,0)
		while count < 406000:
			count += 1
			set_velocity_uav(0.1,0,0,0,0,0)
		while count < 606000:
			count += 1
			set_velocity_uav(0.2,0,0,0,0,0)
		while count < 607000:
			count += 1
			set_velocity_uav(0.1,0,0,0,0,0)
		while count < 608000:
			count += 1
			set_velocity_uav(0,0,0,0,0,0)
		while count < 609000:
			count += 1
			set_velocity_uav(0,-0.1,0,0,0,0)
		while count < 809000:
			count += 1
			set_velocity_uav(0,-0.2,0,0,0,0)
		while count < 900000:
			count += 1
			set_velocity_uav(0,-0.1,0,0,0,0)
		while count < 901000:
			count += 1
			set_velocity_uav(0,0,0,0,0,0)


### Function to Take off to a desired height			
def takeoff_height(height):
	global uav_location
	while(uav_location == 0):
		a=1 
	while(uav_location.position.z < height):
		print uav_location.position.z
		set_velocity_uav(0,0,1.0,0,0,0)
	set_velocity_uav(0.0,0,0,0,0,0)	
    	
### Main Function
if __name__ == '__main__':
	### Rospy Init ####
	rospy.init_node('master', anonymous=True)

	### Subscribe to UAVs Location ###
	uav_location_sub_ekf()

	### Sleep to give world time to build ###
	rospy.sleep(2.)

 	### Take off to given height ###
	takeoff_height(5)

	### Begin Image Processing ###
	image_converter()
	
	### Sleep to give world time to build ###
	rospy.sleep(2.)

	try:
		### Have Mav around the world ###
		mav_controller()
			
		



	except rospy.ROSInterruptException: pass

