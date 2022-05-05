#!/usr/bin/env python
"""
	Sample boiler-plate code for qualifying round
	Cyber Physical System Virtual Organization Challenge 2021 : SoilScope Lunar ExoCam Edition
	Team Name :
	Members :
"""

import rospy
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String, Header, Bool
import math
import time


class OffboardControl:
    """ Controller for PX4-UAV offboard mode """

    def __init__(self):
        rospy.init_node('OffboardControl', anonymous=True)

        # define your class variables here

        self.curr_pose = PoseStamped()  # current pose of the drone
        self.des_pose = PoseStamped()  # desired pose of the drone in position control mode
        self.is_ready_to_fly = False
        self.mode = "ASCEND"
        self.arm = False
        self.chute_detached = False
        self.att = AttitudeTarget()
        self.attach = False
        self.orientation = [0] * 3
        self.attachFlag = True

        # define your services here

        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)

        # define your subscribers and pulishers here

        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pose_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_callback)

        self.parachute_pub = rospy.Publisher('parachute_plugin/sample_probe', Bool, queue_size=10)
        self.att_setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.attach_pub = rospy.Publisher('/probe_attached', Bool, queue_size=10)

        # a function that calls other functions based on its mode
        self.controller()

    def pose_callback(self, msg):
        self.curr_pose = msg
        # gets the euler angles (roll, pitch, yaw) from the quaternion values
        # Note: The angles you get doesn't map the [-pi,pi] range properly and might require some conversion
        self.orientation = euler_from_quaternion(
            (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))

    def state_callback(self, msg):
        if msg.mode != 'OFFBOARD' or self.arm != True:
            # take_off
            self.set_offboard_mode()
            self.set_arm()

    def set_offboard_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    def set_arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
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
        # calling the evaluation code
        self.attach_pub.publish(0)

    def retro(self):
        while self.mode == 'RETRO' and not rospy.is_shutdown():
            # using the following line of code to detach the probe
            self.detach_models('if750a','base_link','sample_probe','base_link')
	    self.attach_pub.publish(Bool(0))
            self.mode = "LAND"

    def land(self):
        rate = rospy.Rate(15)  # Hz
	timer = time.time()
        while self.mode == "LAND" and not rospy.is_shutdown():
            try:
                flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                isModeChanged = flightModeService(custom_mode='AUTO.LAND')
            except rospy.ServiceException as e:
                print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

            # use the following code to pull up the parachute on the probe so that it lands on the spot!
            if time.time()- timer > 1.9:		
                self.parachute_pub.publish(1)

	

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def ascend(self):
        self.des_pose.pose.position.z = 20
        rate = rospy.Rate(15)
        while self.mode == "ASCEND" and not rospy.is_shutdown():
            self.pose_pub.publish(self.des_pose)
            if self.curr_pose.pose.position.z > 0.5 and self.attachFlag:
		
                # Use the following line of code to attach the probe to the drone...
                self.attach_models('if750a', 'base_link', 'sample_probe', 'base_link')
                self.attachFlag = False
	    if self.curr_pose.pose.position.z > 17:
		self.mode = "BELLY-FLOP"
            # if the drone is ready for the throw, change mode to "BELLY-FLOP"
            # self.mode = "BELLL-FLOP"
            rate.sleep()

    def belly_flop(self):

        # add your flip code here

        rate = rospy.Rate(15)  # Hz
        self.set_offboard_mode()
        self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.attach = True

        while self.mode == "BELLY-FLOP" and not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            # use AttitudeTarget.thrust to lift your quadcopter
            self.att.thrust = 0.9
            # use AttitudeTarget.body_rate.y to provide the angular velocity to your quadcopter
            self.att.body_rate.y = 20.0
            # type_mask = 128 is used for controlling the rate exclusively, you may explore other values too
            self.att.type_mask = 128

            if (self.orientation[1] > 0.75):
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
            if self.mode == "ASCEND":
                print("Ascending!")
                self.ascend()
            if self.mode == "BELLY-FLOP":
                print("belly flop!")
                self.belly_flop()
            if self.mode == "RETRO":
                self.retro()
            if self.mode == "LAND":
                self.land()


if __name__ == "__main__":
    OffboardControl()
