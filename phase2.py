#!/usr/bin/env python
"""
	Sample boiler-plate code for phase 1b
	Cyber Physical System Virtual Organization Challenge 2021 : SoilScope Lunar Lander ExoCam -- Earth Analog
	Team Name : Team - 4 Black
	Members : Dhairya Vyas, Ravi Swaroop, Akshit Kalantri, Vikramaditya Nanhai
"""

import rospy
import cv2 
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Vector3Stamped, Quaternion
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget, ActuatorControl, RCOut
from sensor_msgs.msg import Imu, Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String, Header, Bool, Float64
from numpy import sqrt, pi, sin, cos, arctan2, array, linalg, tan, dot
import numpy as np

from std_srvs.srv import Empty
import time

g = 9.80665

class OffboardControl:
	""" Controller for PX4-UAV offboard mode """

	def __init__(self):
		rospy.init_node('OffboardControl', anonymous=True)

		# define your class variables here

		self.curr_pose = PoseStamped()                      # current pose of the drone
		self.des_pose = PoseStamped()                           # desired pose of the drone in position control mode
		self.is_ready_to_fly = False
		self.mode = "ASCEND"
		self.arm = False
		self.chute_detached = False
		self.att = AttitudeTarget()
		self.attach = False
		self.orientation = [0]*3
		self.attachFlag = True

		self.initializeVisionVariables()

		for i in reversed(range(1,4)):
			print "Launching node in {}...".format(i)
			rospy.sleep(1)

		# define ros services, subscribers and publishers here
		# arm or disarm the UAV
		self.armService = rospy.ServiceProxy('/uav/mavros/cmd/arming', CommandBool)
		# attach any two objects in Gazebo
		self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
		# detach any two attached objects
		self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
		# pause the Gazebo simulation if needed (could be used to debug the movement of the UAV)
		self.pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		# example call:
#		self.pause_physics_client.call()
		# could be used to reset the probe if needed
		self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)

		# command your attitude target to the UAV
		self.att_setpoint_pub = rospy.Publisher('/uav/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
		# command a position setpoint to the UAV
		self.pos_setpoint_pub = rospy.Publisher('/uav/mavros/setpoint_position/local', PoseStamped, queue_size=10)
		# publish the debug image for reference if needed
		self.debugImgPub = rospy.Publisher('/debug_cam',Image,queue_size=10)

		# get the current state of the UAV
		self.state_sub = rospy.Subscriber('/uav/mavros/state', State, callback=self.state_callback)
		# get the visual from the onboard camera
		self.img_sub = rospy.Subscriber('/uav_camera_down/image_raw',Image,self.img_cb)

		self.pose_sub = rospy.Subscriber('/uav/mavros/local_position/pose', PoseStamped, callback=self.pose_callback)


		# call the state machine
		self.controller()

	def initializeVisionVariables(self):
		self.bridge = CvBridge()
		self.debug = False
		self.imgSize = array([640,640,3])

	def pose_callback(self, msg):
		self.curr_pose = msg
		# gets the euler angles (roll, pitch, yaw) from the quaternion values
		# Note: The angles you get doesn't map the [-pi,pi] range properly and might require some conversion
		self.orientation = euler_from_quaternion((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))


	def img_cb(self,msg):
		try:
			if self.curr_pose.pose.position.z > 0:
				font = cv2.FONT_HERSHEY_COMPLEX
				# access the visual from 'frame' to get the rover coordinates
				self.pixel_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
				# add your image processing code here
				self.debug = False
				image_blur = cv2.GaussianBlur(self.pixel_img,(3,3),0)  # image blurred
        			image_blur_hsv= cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)
        			# Use the two masks to create double mask
				min_red = np.array([0,0,0])
       				max_red = np.array([40,40,40])
				mask = cv2.inRange(image_blur_hsv, min_red, max_red)
        			edged = cv2.Canny(mask, 200, 500,L2gradient = True )
				result = cv2.bitwise_and(self.pixel_img, self.pixel_img,mask = mask)
        			contours,heirarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:] # Finding contours
				
				# add your image processing code here
				if len(contours) != 0:
					c = max(contours, key = cv2.contourArea) # gives max contour
					M = cv2.moments(c)                       # gives dictionary of all moment values calculated
					if M["m00"]!=0:
					  cX = int(M["m10"]/M["m00"])            # access coordinates for the center
					  cY = int(M["m01"]/M["m00"])
					  approx = cv2.approxPolyDP(c, 0.009 * cv2.arcLength(c, True), True)  #approximation of contour shape
					  cv2.drawContours(self.pixel_img, [approx], 0, (0, 0, 255), 5)  
					  cv2.circle(self.pixel_img,(cX,cY),7,(0,0,255), -1)
					  string = str(cX) + " " + str(cY)
					  cv2.putText(self.pixel_img,string,(cX,cY),font,0.5,(255,0,0),2) # put text
    					  
			                y_a = 320-(cY)  # cY is a center of rover in Y direction, and 320 is the center location of drone.
			                x_a = 320-(cX)  # cX is a center of rover in X direction, and 320 is the center location of drone.
			                self.prob_ang = np.degrees(np.arctan2(y_a,x_a))  #formula to find angle between drone and rover
                		cv2.imshow("Image_window",self.pixel_img)
                		cv2.waitKey(3)
			if self.debug:
				# could be used to debug your detection logic
				data = self.bridge.cv2_to_imgmsg(frame,"bgr8")
				data.header.stamp = msg.header.stamp
				self.debugImgPub.publish(data)
		except CvBridgeError as e:
#			print(e)
			pass

	


	def state_callback(self, msg):
		if msg.mode != 'OFFBOARD' or self.arm != True:
			# take_off
			self.set_offboard_mode()
			self.set_arm()

	def set_offboard_mode(self):
		rospy.wait_for_service('/uav/mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('/uav/mavros/set_mode', SetMode)
			isModeChanged = flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException as e:
			print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

	def set_arm(self):
		rospy.wait_for_service('/uav/mavros/cmd/arming')
		try:
			self.armService = rospy.ServiceProxy('/uav/mavros/cmd/arming', CommandBool)
			self.armService(True)
			self.arm = True
		except rospy.ServiceException as e:
			print("Service arm call failed: %s" % e)



	def attach_models(self, model1, link1, model2, link2):
		req = AttachRequest()
		req.model_name_1 = model1
		req.link_name_1 = link1
		req.model_name_2 = model2
		req.link_name_2 = link2
		self.attach_srv.call(req)

	def detach_models(self, model1, link1, model2, link2):
		req = AttachRequest()
		req.model_name_1 = model1
		req.link_name_1 = link1
		req.model_name_2 = model2
		req.link_name_2 = link2
		self.detach_srv.call(req)


	def retro(self):
		while self.mode == 'RETRO' and not rospy.is_shutdown():
			# using the following line of code to detach the probe
			self.detach_models('if750a_1','base_link','sample_probe','base_link')
			self.mode = "LAND"
			

	def land(self):
		rate = rospy.Rate(15)  # Hz
		while self.mode == "LAND" and not rospy.is_shutdown():
			try:
				flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
				isModeChanged = flightModeService(custom_mode='AUTO.LAND')
			except rospy.ServiceException as e:
				print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

			

			try:  # prevent garbage in console output when thread is killed
				rate.sleep()
			except rospy.ROSInterruptException:
				pass

	def ascend(self):
		self.des_pose.pose.position.z = 10
		rate = rospy.Rate(15)
		rate = rospy.Rate(15)
		att_reach = False
		#t = time.time()
		while self.mode=="ASCEND" and not rospy.is_shutdown():
			self.pos_setpoint_pub.publish(self.des_pose)
			if self.curr_pose.pose.position.z>0.5 and self.attachFlag:
				# Use the following line of code to attach the probe to the drone...
				self.attach_models('if750a_1','base_link','sample_probe','base_link')
				self.attachFlag = False
			if self.curr_pose.pose.position.z > 10:
				if att_reach is False:
					att_reach = True
				if att_reach is True and 114 <= self.prob_ang <=115:
					self.mode = "BELLY-FLOP"
			rate.sleep()

	def belly_flop(self):
		rate = rospy.Rate(15)  # Hz
		self.set_offboard_mode()
		self.att.body_rate = Vector3()
		self.att.header = Header()
		self.att.header.frame_id = "base_footprint"
		self.attach = True

		while self.mode == "BELLY-FLOP" and not rospy.is_shutdown():
			self.att.header.stamp = rospy.Time.now()
			# use AttitudeTarget.thrust to lift your quadcopter
			self.att.thrust = 1
			# use AttitudeTarget.body_rate.y to provide the angular velocity to your quadcopter
			self.att.body_rate.y = 9.0
			# type_mask = 128 is used for controlling the rate exclusively, you may explore other values too
			self.att.type_mask = 128
			if self.orientation[1]>0.56:
			    self.att_setpoint_pub.publish(self.att)
			    rate.sleep()
			    self.mode="RETRO"

			self.att_setpoint_pub.publish(self.att)

			try:  # prevent garbage in console output when thread is killed
				rate.sleep()
			except rospy.ROSInterruptException:
				pass



	def controller(self):
		""" A state machine developed to have UAV states """
		while not rospy.is_shutdown():
			# control your UAV states and functionalities here...
			if self.mode =="ASCEND":
				print("Ascending!")
				self.ascend()
			if self.mode =="BELLY-FLOP":
				print("belly flop!")
				self.belly_flop()
			if self.mode == "RETRO":
				self.retro()
			if self.mode == "LAND":
				self.land()


if __name__ == "__main__":
	
	OffboardControl()
