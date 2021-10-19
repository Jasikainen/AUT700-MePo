#!/usr/bin/env python3

import rospy
import sys

from geometry_msgs.msg  import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import * # import all just in case

from math import pow, atan2, sqrt

HALF_TURN = math.pi # As a constant

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
		self.rate = rospy.Rate(60)
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
		print("theta  degrees: %f  rads: %f" % (theta, self.target_theta))
		rospy.sleep(4)


	def ask_user_for_point(self):
		"""
		Function to ask for user input on some scenarios
		"""
		print("Please give following information to proceed")
		# Define Points x,y components to be the target for turtlebot3
		x = float(input("Point 'x': "))
		y = float(input("Point 'y': "))
		theta = float(input("Orientation as degrees 'θ': "))
		self.set_pose(x, y, theta)
		
		
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
			rospy.loginfo("1st")
			return steering_needed - self.theta
			
		elif steering_needed < 0 and self.theta > 0:
			# e.g. steering_needed = -2.9 rads, current 2.9 rads -> calculate difference and add full turn
			rospy.loginfo("2nd  steering_needed: %f  self.theta: %f" % (steering_needed, self.theta))
			angle_to_rotate = (steering_needed - self.theta) + 2.0 * HALF_TURN
			return angle_to_rotate if angle_to_rotate < HALF_TURN else -1.0*(2.0 * HALF_TURN - angle_to_rotate)

			
		elif steering_needed < 0 and self.theta < 0 and steering_needed < self.theta:
			# e.g. steering_needed = -2.8 rads and theta = -2.5 rads -> sum theta and get +0.3
			rospy.loginfo("3rd")
			return steering_needed + self.theta
			
		elif steering_needed > 0 and self.theta < 0:
			angle_to_rotate = steering_needed - self.theta
			rospy.loginfo("4th")
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

		rospy.loginfo("angle_to_rotate %f" % angle_to_rotate)
		
		# Copy sign of corrected_angle to waffle_pi's maximum angular velocity
		angular_velocity = self.angular_controls_performed * angle_to_rotate
		return max(angular_velocity, math.copysign(1.82, angle_to_rotate)) if angle_to_rotate < 0 \
		 	else min(angular_velocity, 1.82)
		

	def move2point(self):
		"""
		Task 2.2: 5
		Function to move turtlebot to given position (x,y) in workspace
		"""
    
		# Define Twist to control velocity that will be pusblihed to /cmd_vel topic
		vel_msg = Twist()

		# Loop that runs until given point (x,y) is achieved within specific tolerance
		while self.euclidean_distance() >= self.dist_tol:
			rospy.loginfo("-----------------------------------")
			rospy.loginfo("Distance to goal: %s" % self.euclidean_distance())
			rospy.loginfo("Turtlebot3:  Pose (x, y, θ) = (%3f, %3f, %3f)" % (self.x, self.y, self.theta))
			
			rospy.loginfo("Velocity:  Angular: %3f   Linear: %3f" % (self.angular_z, self.linear_x))
			
			steering_angle = self.steering_angle()
			# NOTE: Theta is possibly -pi, pi values so -pi values equal to >180 deg values as in steering angle
			# Convert self.theta to 0 to 360 degrees
			angle_to_point = self.angle_to_goal()
			rospy.loginfo("Steering Angle : %3f  Angle To Goal: %3f" % (steering_angle, angle_to_point))
			
			# Use angle_to_goal compared to 5 deg tolerance as a decider of control to be used
			if abs(angle_to_point) > math.pi/180.0 * 10:
				# move forward if angle to goal (10, 35)
				vel_msg.linear.x = self.linear_velocity() if abs(angle_to_point) < math.pi/180.0 * 35 else 0.0
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
				vel_msg.angular.z = 0.0 if abs(angle_to_point) < math.pi/180.0 * 5 else self.angular_velocity()		

			# Publish into /cmd_vel topic :)
			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()

		rospy.loginfo("Turtlebot is now at pose: (x, y, θ)=(%2f, %2f, %2f°)" % (self.x, self.y, math.degrees(self.theta)))
		# if while loop cond is not satisfied stop the bot
		vel_msg.linear.x = 0.0
		vel_msg.angular.z = 0.0
		self.velocity_publisher.publish(vel_msg)
		rospy.sleep(2)

	
	def rotate2orientation(self):
		"""
		Task 2.2: 5
		Function to rotate turtlebot to given orientation after reaching position (x,y)
		"""

		# Define Twist to control velocity that will be pusblihed to /cmd_vel topic
		vel_msg = Twist()

		# Loop that runs until given orientation is in range of 1 deg to target
		while abs(self.target_theta - self.theta) > (HALF_TURN/180.0)*0.5:
			rospy.loginfo("Target rads : %f2  Bot rads: %2f  Angular Velocity: %2f" % (self.target_theta, self.theta, self.angular_z))
			vel_msg.linear.x = 0.0
			vel_msg.linear.y = 0.0
			vel_msg.linear.z = 0.0
		
			vel_msg.angular.x = 0.0
			vel_msg.angular.y = 0.0
			vel_msg.angular.z  = 0.8 * (self.target_theta - self.theta)
			
			# Publish into /cmd_vel topic :)
			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()
			
		rospy.loginfo("Turtlebot is now at pose: (x, y, θ)=(%2f, %2f, %2f°)" % (self.x, self.y, math.degrees(self.theta)))
		vel_msg.linear.x = 0.0
		vel_msg.angular.z = 0.0
		self.velocity_publisher.publish(vel_msg)


def continue_or_stop(question):
	"""
	n or y is parsed from inputs first character so it's the only part that matters
	"""
	while "answer not y or n":
		reply = str(input(question + " (y/n): " )).lower().strip()
		if reply[:1] == 'y':
			return True
		elif reply[:1] == 'n':
			return False
		else:
			print("ERROR: Not a proper prompt")


if __name__ == '__main__':
	try:
		turtle_bot = Turtlebot()
		if len(sys.argv) == 4:
			x = float(sys.argv[1])
			y = float(sys.argv[2])
			theta = float(sys.argv[3])
			
			turtle_bot.set_pose(x, y, theta)
			turtle_bot.move2point()
			turtle_bot.rotate2orientation()
		else:
			turtle_bot.ask_user_for_point()
			turtle_bot.move2point()
			turtle_bot.rotate2orientation()
		
		while True:
			if not continue_or_stop("Do you want to enter a new target"):
				break
			turtle_bot.ask_user_for_point()
			turtle_bot.move2point()
			turtle_bot.rotate2orientation()
			
	except rospy.ROSInterruptException: 
		pass
	except ValueError:
		print("Either of the input values was not convertible to float(): %s %s %s" \
			% (sys.argv[1], sys.argv[2], sys.argv[3]))
		sys.exit(1)

