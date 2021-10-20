#!/usr/bin/env python3

import rospy
import sys

from geometry_msgs.msg  import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import * # import all just in case

from math import pow, atan2, sqrt

HALF_TURN = math.pi # As a constant

# Pairs of coordinates to travel through
square_path_xy = [(3.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)]


class Turtlebot():
	def __init__(self):
		# Creating our node,publisher and subscriber
		rospy.init_node('turtlebot3_controller', anonymous=True)

		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odom)
		
		# Publish to /cmd_vel
		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		# Create a Point for target that may be modified later on
		self.target_point = Point ()
		self.target_theta = 0.0
		
		# This includes x, y and theta from "odom.pose.pose.position..."
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		
		# This below will have the current angular speed (z-axis) of turtlebot3
		self.angular_z = 0.0		
		self.linear_x = 0.0
		
		# Use this value to control the start of new control linear_velocity control sequence
		self.linear_controls_performed = 0.0
		self.angular_controls_performed = 0.0
		
		# Use 20 Hz pub rate
		self.rate = rospy.Rate(20)
		# Default tolerance used
		self.dist_tol = 0.05


	def get_odom(self, msg):
		"""
		Callback function for handling messages got from topic /odom thats been subscribed to
		"""
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		self.angular_z = msg.twist.twist.angular.z
		self.linear_x = msg.twist.twist.linear.x
		
		# Get quaternion from msg
		ori_q = msg.pose.pose.orientation # x, y, z, w
		# roll and pitch not used. NOTE: yaw == theta
		# theta values are between 180, -180 deg <=> pi, -pi
		(roll, pitch, self.theta) = euler_from_quaternion([ori_q.x, ori_q.y, ori_q.z, ori_q.w])

	
	def get_xy(self):
		return (self.x, self.y)
	
	def set_pose(self, x, y, theta):
		"""
		Function to set pose to be achieved - (x,y,theta)
		
		e.g. positive values
		* 175 % 360 = 175 -> OK, 185 % 360 = 185 > pi -> 185-360 = -175
		* (175+n*360) % 360 = 175 -> OK, (185+n*360) % 360 = 185 > pi -> 185-2pi = -175
		
		e.g. negative values - in this the values overlapping half turn work opposite way
		* -185 % 360 = 175 -> OK, -175 % 360 = 185 > pi -> 185-2pi = -175
		* (-185-n*360) % 360 = 175 -> OK, (-175-n*360) = 185 > pi -> 185-2pi = -175
		"""
		self.target_point.x = x
		self.target_point.y = y
		# Calculate target orientation and set as rads
		theta = theta % 360.0
		#rospy.loginfo("theta after modulo %f" % theta)
		theta = theta if (theta < 180.0) else (theta - 360.0)
		#rospy.loginfo("theta after conversion %f" % theta)
		self.target_theta = (HALF_TURN/180.0) * theta
		print("Next pose: (x,y) : (%f, %f)  Orientation: %f" % (self.target_point.x, self.target_point.y, theta))
		rospy.sleep(2)
		
		
	def euclidean_distance(self):
		"""
		Function to calculate euclidian distance between given point and 
		classes current position in x,y coordinates
		
		return: euclidean distance between target and current coordinates 
		(length of line segment between two points in 2D space)
		"""
		return sqrt(pow((self.target_point.x - self.x), 2) + pow((self.target_point.y - self.y), 2))


	def linear_velocity(self):
		"""
		Function to calculate linear velocity by using euclidean distance as the scaling factor
		param: multiplier is set to lower value because euclidean distance itself will be greater
		
		return: linear velocity to be set
		"""
		self.linear_controls_performed += 0.4
		return min(self.linear_controls_performed * self.euclidean_distance(), 0.22)


	def coordinate_difference(self):
		"""
		Function to calculate difference between target point coords versus current
		
		return: absolute difference between target and current coordinates
		"""
		return self.target_point.x - self.x, self.target_point.y - self.y

	
	def steering_angle(self):
		"""
		Function to calculate angle between given coords and current coords by calculating atan2(args)
		
		return: Angle from current coordinate to goal as in unit circle
		"""
		# Calculate the differences in x and y coordinates from Odometry to target Point
		(diff_x, diff_y) = self.coordinate_difference()
		
		# math.atan2 returns values from pi to -pi radians as default
		return atan2(diff_y, diff_x)


	def angle_to_goal(self):
		steering_needed = self.steering_angle()
		
		# Both positive and automatically correct signed result between
		if (steering_needed > 0 and self.theta > 0) \
				or (steering_needed < 0 and self.theta < 0 and steering_needed > self.theta):
			return steering_needed - self.theta
			
		elif steering_needed < 0 and self.theta > 0:
			# e.g. steering_needed = -2.9 rads, current 2.9 rads -> calculate difference and add full turn
			angle_to_rotate = (steering_needed - self.theta) + 2.0 * HALF_TURN
			return angle_to_rotate if angle_to_rotate < HALF_TURN else -1.0*(2.0 * HALF_TURN - angle_to_rotate)

			
		elif steering_needed < 0 and self.theta < 0 and steering_needed < self.theta:
			# e.g. steering_needed = -2.8 rads and theta = -2.5 rads -> sum theta and get +0.3
			return steering_needed + self.theta
			
		elif steering_needed > 0 and self.theta < 0:
			angle_to_rotate = steering_needed - self.theta
			# if angle_to_rotate > pi negative turn for "less half turn CW" (-6.283 - 1.6 - (-1.6) = -3.083)
			# else angle_to_rotate < pi turn positive direction for the value calculated
			return angle_to_rotate if angle_to_rotate < HALF_TURN else -1.0*(HALF_TURN*2.0 - angle_to_rotate)
		else:
			return steering_needed - self.theta
			
	def angular_velocity(self):
		"""
		Function to calculate angular velocity by using euclidian distance as the scaling factor
		param: multiplier is set to greater as default because the angle needed to rotate towards target
		in radians will be most likely quite small
		
		return: angular velocity
		"""
		self.angular_controls_performed += 0.15
		angle_to_rotate = self.angle_to_goal()
		
		# Copy sign of corrected_angle to waffle_pi's maximum angular velocity
		angular_velocity = self.angular_controls_performed * angle_to_rotate
		return max(angular_velocity, math.copysign(1.82, angle_to_rotate)) if angle_to_rotate < 0 \
		 	else min(angular_velocity, 1.82)
		

	def move2point(self):
		"""
		Task 2.2: 5
		Function to move turtlebot to given position (x,y) in workspace
		"""
		vel_msg = Twist()

		# Loop that runs until given point (x,y) is achieved within specific tolerance
		while self.euclidean_distance() >= self.dist_tol:
			print("-----------------------------------")
			print("Distance to goal: %s" % self.euclidean_distance())
			print("Velocity:  Angular: %f   Linear: %f" % (self.angular_z, self.linear_x))
			
			steering_angle = self.steering_angle()
			# NOTE: Theta is possibly -pi, pi values so -pi values equal to >180 deg values as in steering angle
			# Convert self.theta to 0 to 360 degrees
			angle_to_point = self.angle_to_goal()
			print("Angles:  Steering: %f  ToGoal: %f" % (steering_angle, angle_to_point))
			
			# Use angle_to_goal compared to 5 deg tolerance as a decider of control to be used
			if abs(angle_to_point) > math.pi/180.0 * 0.5:
				# move forward if angle to goal (5, 15)
				vel_msg.linear.x = self.linear_velocity() if \
					abs(angle_to_point) < math.pi/180.0 * 10 else 0.0
				vel_msg.linear.y = 0.0
				vel_msg.linear.z = 0.0
			
				vel_msg.angular.x = 0.0
				vel_msg.angular.y = 0.0
				self.linear_controls_performed = 0.0
				vel_msg.angular.z  = self.angular_velocity()
			else:
				vel_msg.linear.x = self.linear_velocity()
				vel_msg.linear.y = 0.0
				vel_msg.linear.z = 0.0

				vel_msg.angular.x = 0.0
				vel_msg.angular.y = 0.0
				self.angular_controls_performed = 0.0
				vel_msg.angular.z = 0.0 if abs(angle_to_point) < math.pi/180.0 * 1 else self.angular_velocity()		

			# Publish into /cmd_vel topic :)
			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()
		# Goal reached
		rospy.loginfo("Goal reached! Turtlebot is now at: (x, y, θ) = (%f, %f, %f°)" % (self.x, self.y, math.degrees(self.theta)))
		vel_msg.linear.x = 0.0
		vel_msg.angular.z = 0.0
		self.velocity_publisher.publish(vel_msg)

	
	def rotate2orientation(self):
		"""
		Task 2.2: 5
		Function to rotate turtlebot to given orientation after reaching position (x,y)
		"""
		
		# Define Twist to control velocity that will be pusblihed to /cmd_vel topic
		vel_msg = Twist()

		# Loop that runs until given orientation is in range of 0.5 deg to target
		while abs(self.target_theta - self.theta) > (HALF_TURN/180.0)*0.5:
			print("-----------------------------------")
			print("Target rads : %f  Bot rads: %f" % (self.target_theta, self.theta))
			print("Velocity:  Angular: %f" % self.angular_z)
			vel_msg.linear.x = 0.0
			vel_msg.linear.y = 0.0
			vel_msg.linear.z = 0.0
		
			vel_msg.angular.x = 0.0
			vel_msg.angular.y = 0.0
			vel_msg.angular.z  = 0.8 * (self.target_theta - self.theta)
			
			# Publish into /cmd_vel topic :)
			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()
			
		rospy.loginfo("Turtlebot is now at pose: (x, y, θ) = (%2f, %2f, %2f°)" % (self.x, self.y, math.degrees(self.theta)))
		vel_msg.linear.x = 0.0
		vel_msg.angular.z = 0.0
		self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
	try:
		turtle_bot = Turtlebot()
		# Wait for a second so that msg in /odom topic gets published and turtlebot gets correct initial position
		rospy.sleep(1) 
		
		# Get initial position and tell it to user
		init_x, init_y = turtle_bot.get_xy()
		init_theta = (atan2(0.0 - init_x, 0.0 - init_y)) * 180.0/HALF_TURN
		print("Initial position X: %f  Y: %f" % (init_x, init_y))
		rospy.sleep(1)
		
		# Set Origo to be starting point - theta is not necessary though on this case
		turtle_bot.set_pose(0.0, 0.0, init_theta)
		turtle_bot.move2point()
		
		for x, y in square_path_xy:
			# Set the target pose by current coords and future coords.
			turtle_x, turtle_y = turtle_bot.get_xy()
			theta = (atan2(y - turtle_y, x - turtle_x)) * 180.0/HALF_TURN
			turtle_bot.set_pose(x, y, theta)
			
			# Rotate turtlebot towards goal for more accurate tracking
			turtle_bot.rotate2orientation()
			# Move forwards the next target goal
			turtle_bot.move2point()
		
	except rospy.ROSInterruptException: 
		pass

