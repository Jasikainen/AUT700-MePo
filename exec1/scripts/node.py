#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import String
from sensor_msgs.msg import JointState # Subscribed to
from nav_msgs.msg import Odometry # Used for publishing
from geometry_msgs.msg import Twist, Pose # Used in Odometry

from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Header
from tf.transformations import * # import all just in case
import rosbag

# rosbag play ~/Jointstate.bag

class Node():
	def __init__(self):
		# Create subscriber to joint_state topic
		self.jointstate_subscriber = rospy.Subscriber('/joint_states', JointState, self.get_joint_state)

		# Create publisher to odometry topic. This is the data created from joint_state topic.
		self.odometry_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
		
		self.tf_broadcaster = TransformBroadcaster()

		# Initial state of the right and left wheel. These are updated via get_joint_state
		self.current_right_wheel_position = 0.0
		self.current_left_wheel_position = 0.0

		self.prior_right_wheel_position = 0.0
		self.prior_left_wheel_position = 0.0

		# units: m
		self.WHEEL_RADIUS = 0.1
		self.WHEEL_DISTANCE = 0.4

		# Linear and angular velocity of imaginary robot
		self.v_x = 0.0
		self.w_z = 0.0

		# Initial conditions for x, y, 𝜓 (psi)
		self.x_t_prior = 0.0
		self.y_t_prior = 0.0
		self.psi_t_prior = 0.0

		# Current and last time will be used to deriving time
		self.current_time = rospy.get_time()
		self.last_time = rospy.get_time()

		# use 1 Hz rate 
		self.rate = rospy.Rate(10)


	def get_joint_state(self, msg):
		"""
		This is a callback function for "/jointstate" subscriber.
		Takes the wheel positions and sets them to Nodes member variables
		"""
		# Set current states to prior states before update
		self.prior_right_wheel_position = self.current_right_wheel_position
		self.prior_left_wheel_position = self.current_left_wheel_position
		# Update current states from joint_state message
		self.current_right_wheel_position = msg.position[0]
		self.current_left_wheel_position = msg.position[1]


	def listener(self):
		# Define Odometry to data that will be published to /odom topic
		odom_msg = Odometry()
		header = Header()
		header.frame_id = 'Odom'
		odom_msg.header = header
		odom_msg.child_frame_id = 'robot_base'

		# Loop that runs until program has been stopped with ctrl + C
		while not rospy.is_shutdown():
			# Set new current time on each iteration to calculate difference of time
			self.current_time = rospy.get_time()
			delta_time = rospy.get_time()
			delta_time = (self.current_time - self.last_time)

			# Calculate differences in wheel positions (radians)
			delta_right_wheel = self.current_right_wheel_position - self.prior_right_wheel_position # rad
			delta_left_wheel = self.current_left_wheel_position - self.prior_left_wheel_position    # rad

			# Calculate wheel velocities
			v_right = self.WHEEL_RADIUS * (delta_right_wheel / delta_time) # m * rad/s 
			v_left = self.WHEEL_RADIUS * (delta_left_wheel / delta_time)   # m * rad/s

			# Linear velocity (m/s)
			self.v_x = 1/2 * (v_right + v_left)
			# Angular velocity
			self.w_z = 1/self.WHEEL_DISTANCE * (v_right - v_left)

			# Update odometry with velocities
			twist_input = Twist()
			twist_input.linear.x = self.v_x
			twist_input.angular.z = self.w_z
			odom_msg.twist.twist = twist_input

			# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
			# As we now know the robots linear and angular velocity   #
			# We can use Euler method to estimate trajectory of robot #
			# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

			# Generate odometry increment parameters
			delta_trans = self.v_x * delta_time
			delta_rot = self.w_z * delta_time

			# Psi hat variable changes whether we use "regular" or "mid point" method
			# This change only has effect on the x_t and y_t values calculated
			# psi_t_hat = self.psi_t_prior # forward euler method
			psi_t_hat = self.psi_t_prior + 1/2 * delta_rot # Mid point method

			# Calculate new x_t, y_t and psi_t which will later replace prior values
			x_t = self.x_t_prior + delta_trans * math.cos(psi_t_hat)
			y_t = self.y_t_prior + delta_trans * math.sin(psi_t_hat)
			psi_t = self.psi_t_prior + delta_rot # This is always equal like equation
			
			# Update Pose with calculated values
			pose_input = Pose()
			pose_input.position.x = x_t
			pose_input.position.y = y_t
			pose_input.position.z = 0.0

			# Basically this is from RPY that is rotation around X, Z and Y axises
			# We need last one as mobile robot rotates around Z
			quaternion = quaternion_from_euler(0, 0, psi_t)
			pose_input.orientation.x = quaternion[0]
			pose_input.orientation.y = quaternion[1]
			pose_input.orientation.z = quaternion[2]
			pose_input.orientation.w = quaternion[3]
			odom_msg.pose.pose = pose_input
			
			rospy.loginfo("v_x: %f\nw_z: %f" % (self.v_x, self.w_z))
			

			self.odometry_publisher.publish(odom_msg)
			# Publish target transform
			self.tf_broadcaster.sendTransform((x_t, y_t, 0.0), quaternion, 
				rospy.Time.now(), "robot_base", "Odom")

			# Set current values to be prior
			self.x_t_prior = x_t
			self.y_t_prior = y_t
			self.psi_t_prior = psi_t

			self.last_time = self.current_time
			# Sleep on each loop
			self.rate.sleep()


def main():
	# First initialize the node
	rospy.init_node('node1')
	rospy.loginfo("Program has started and continues in 2 seconds!")
	rospy.sleep(2)
	node = Node()
	node.listener()
	rospy.loginfo("Program has ended")

if __name__ == '__main__':
	try:
		main()
		
	except rospy.ROSInterruptException:
		pass