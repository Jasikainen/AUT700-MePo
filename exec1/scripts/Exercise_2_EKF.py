#!/usr/bin/env python

from click import FLOAT
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovariance
import tf, math
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
import numpy as np
import matplotlib.pyplot as plt 

# Exercise 1. answer is used as an base for exercise 2. 2) 
#Extended Kalman Filter

# All vectors are in column form which is as (n,) in numpy. x and y particle results
# are stacked column wise together -> columns of x and y matrices are then paired in graphs.

# Jani Asikainen
# Henri Pulkkinen

# NOTE: add exec1/Exercise_2_EK.py file inside CMakeLists.txt file of package

# 1. run roscore to get things working
# roscore

# 2. Startup the node
# rosrun exec1 Exercise_2_MC.py

# 3. Start recording odometry messages
# rosbag record -o odometry_ex2 odometry

# 4. play the Exercise-2.bag file that is listened to by ROS node
# rosbag play ~/Exercise-2.bag

# -------------------------------------


pub = rospy.Publisher('posewithcovariance', PoseWithCovariance, queue_size=10)
pose_with_cov = PoseWithCovariance()
pose_with_cov.header.frame_id = "Odom"
pose_with_cov.child_frame_id = "robot_base"

prev_wheel_pos = [0, 0]
prev_t = 0
cumulative_t = 0.0 # keeps track of cumulative time. Determines when to store states of particles.
start_time = 0

# Preallocate to store previous states
# subscript: t-1
x_t_prev = 0 # (100,)
y_t_prev = 0
rz_t_prev = 0 

# Preallocate to store new states in calculation
# subscript: t
x_t = 0
y_t = 0
rz_t = 0


x_t_storage = np.array([]) # initialize empty for storaging states of particles
y_t_storage = np.array([])
t_cumulative_storage = np.array([])

# Locations of known landmarks in the world
#                   X      Y    Z    R    P    Y
landmarks = {
            "L0": [ 7.5, -4.0, 0.0, 0.0, 0.0, 0.0],
            "L1": [-5.0,  8.0, 0.0, 0.0, 0.0, 0.0],
            "L2": [-7.0, -6.5, 0.0, 0.0, 0.0, 0.0]
            }

# Create matrices for extended kalman filtering that are Jacobians
# Derivate respect to different variables
A = np.array([])
H = np.array([])
L = np.array([])
# constants
wheel_radius = 0.1
dist_bw_wheels = 0.4


def callback(data):
    # Global definitions as these are changed during execution
    global prev_t, prev_wheel_pos, rz_t_prev, x_t_storage, y_t_storage, \
        cumulative_t, x_t_prev, y_t_prev, start_time, rz_t_avg_storage, x_t_avg_storage, y_t_avg_storage

    # Update times
    new_t = data.header.stamp.to_sec()
    dt = new_t - prev_t
    prev_t = new_t

    # Bad time stamps -> do calibration this way to get real cumulative time:)
    if cumulative_t == 0.0:
        start_time = new_t

    # add time difference -> check later when to store states of particles (x,y)
    cumulative_t = cumulative_t + dt

    # Calculating robot velocity--------------------------------------

    new_wheel_pos = data.position
    right_vel = (new_wheel_pos[0] - prev_wheel_pos[0]) / dt
    left_vel = (new_wheel_pos[1] - prev_wheel_pos[1]) / dt

    # Set previous wheel position for next iteration
    prev_wheel_pos = new_wheel_pos

    # Simply calculate linear and angular velocity from wheel velocities
    vr = wheel_radius * right_vel
    vl = wheel_radius * left_vel
    vx = 0.5 * (vr + vl)
    wz = (1 / dist_bw_wheels) * (vr - vl)

    # Calculate sample predictions, inputs have been corrupted by noise
    delta_hat_trans = vx * dt
    delta_hat_rot_1 = 1/2 * wz * dt
    delta_hat_rot_2 = delta_hat_rot_1

    # Form noise to each of the rotation and translation factors
    # Noises are zero mean with variances calculated as follows. 
    sigma_1 = a1 * math.pow(delta_hat_rot_1, 2)  +  a2 * math.pow(delta_hat_trans, 2)

    sigma_2 = a3 * math.pow(delta_hat_trans, 2)  +  a4 * (math.pow(delta_hat_rot_1, 2) + math.pow(delta_hat_rot_2, 2))

    sigma_3 = a1 * math.pow(delta_hat_rot_2, 2)  +  a2 * math.pow(delta_hat_trans, 2)

    # Draw 100 values from noise distribution
    epsilon_rot_1 = np.random.normal(0, sigma_1, draw_samples) # shape (100,)
    epsilon_trans = np.random.normal(0, sigma_2, draw_samples) 
    epsilon_rot_2 = np.random.normal(0, sigma_3, draw_samples)

    # Helper variables for predictions that have - (100,)
    delta_rot_1 = delta_hat_rot_1 + epsilon_rot_1
    delta_trans = delta_hat_trans + epsilon_trans
    delta_rot_2 = delta_hat_rot_2 + epsilon_rot_2

    # Calculate sample predictions
    x_t = np.add(x_t_prev, np.multiply(delta_trans, np.array(np.cos(rz_t_prev + delta_rot_1))))
    y_t = np.add(y_t_prev, np.multiply(delta_trans, np.array(np.sin(rz_t_prev + delta_rot_1))))
    rz_t_prev = rz_t_prev + delta_rot_1 + delta_rot_2 # all particles rz_t_prev does not need to be stored for plotting


    # Broadcasting TF---------------------------------------------------
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "Odom"
    t.child_frame_id = "robot_base_kf"
    t.transform.translation.x = odom.pose.pose.position.x
    t.transform.translation.y = odom.pose.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = quat_array[0]
    t.transform.rotation.y = quat_array[1]
    t.transform.rotation.z = quat_array[2]
    t.transform.rotation.w = quat_array[3]

    br.sendTransform(t)
    rospy.loginfo("Publishing odometry and TF.....................")

    # Update previous x and y for next iteration
    x_t_prev = x_t
    y_t_prev = y_t

    
def extended_kalman_filter():
    rospy.init_node('wheel_odometry', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.loginfo("Extended Kalman Filtering has started")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def plot_result(case):
    """
    This function is used to plot the results of exercise
    """
    if case == 1:
        print(np.shape(x_t_storage))


if __name__ == '__main__':
    plot_result(1)
    extended_kalman_filter()
    print(f'program ended by user action: CTRL + C')
    print(np.shape(x_t_storage))
    print(np.shape(y_t_storage))
    plot_result(2)