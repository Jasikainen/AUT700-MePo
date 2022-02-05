#!/usr/bin/env python

from click import FLOAT
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf, math
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
import numpy as np
import matplotlib.pyplot as plt 

# Exercise 1. answer is used as an base for exercise 2. 2) 
# Extended Kalman Filter

# Jani Asikainen
# Henri Pulkkinen

# NOTE: add exec1/Exercise_2_EK.py file inside CMakeLists.txt file of package

# 1. run roscore to get things working
# roscore

# 2. Startup the node
# rosrun exec1 Exercise_2_MC.py

# 4. play the Exercise-2.bag file that is listened to by ROS node
# rosbag play ~/Exercise-2-ekf.bag

# -------------------------------------
pub = rospy.Publisher('posewithcovariance', PoseWithCovarianceStamped, queue_size=10)
pose_with_cov = PoseWithCovarianceStamped()
pose_with_cov.header.frame_id = "Odom"

# Some basics needed for program to work
prev_wheel_pos = [0, 0]
prev_t = 0
cumulative_t = 0.0 # keeps track of cumulative time. Determines when to store states of particles.
start_time = 0
# Preallocate to store previous states
# subscript: t-1
x_t1_pos = 0 # (100,)
y_t1_pos = 0
rz_t1_pos = 0 
# Preallocate to store new states in calculation
# subscript: t
x_t_neg = 0
y_t_neg = 0
rz_t_neg = 0

# Preallocate to due to need in matrices later at definitions
delta_hat_trans = 0
delta_hat_rot_1 = 0
delta_hat_rot_2 = 0
# initialize empty arrays for storaging states to plot at the end of program
x_t_storage = np.array([]) 
y_t_storage = np.array([])
rot_z_storage = np.array([])
t_cumulative_storage = np.array([]) # for plotting

# Create matrices for extended kalman filtering that are Jacobians
# Derivate respect to different variables

# A_k-1 = jacobian of f(X) respect to x, y and rz.
def get_A():
    global rz_t1_pos, delta_hat_rot_1, delta_hat_trans
    # Uses x_hat_t-1 values
    A_k_1 = np.array
    ([
        [1, 0, -delta_hat_trans*math.sin(rz_t1_pos + delta_hat_rot_1)],
        [0, 1,  delta_hat_trans*math.cos(rz_t1_pos + delta_hat_rot_1)],
        [0, 0, 1]
    ])
    return A_k_1

# L_k-1 = jacobian of f(X) respect to eps_trans, eps_rot1 and eps_rot2.
def get_L():
    global rz_t1_pos, delta_hat_rot_1, delta_hat_trans
    # Uses t-1 values
    L = np.array
    ([
        [math.cos(rz_t1_pos + delta_hat_rot_1), -delta_hat_trans*math.sin(rz_t1_pos + delta_hat_rot_1), 0],
        [0, 1,  delta_hat_trans*math.cos(rz_t1_pos + delta_hat_rot_1)],
        [0, 0, 1]
    ])
    return L


# -----------------------
# Stuff related to measurements and localization

# Locations of known LANDMARKS in the world
LANDMARKS = \
{
    "L0": [ 7.5, -4.0, 0.0, 0.0, 0.0, 0.0],
    "L1": [-5.0,  8.0, 0.0, 0.0, 0.0, 0.0],
    "L2": [-7.0, -6.5, 0.0, 0.0, 0.0, 0.0]
}
L0 = LANDMARKS["L0"] # X Y Z R P Y
L1 = LANDMARKS["L1"] # X Y Z R P Y
L2 = LANDMARKS["L2"] # X Y Z R P Y

# H_k = jacobian of h(X) respect to x, y and rz.
# h(x,y) = sqrt((landmark_x[i] - x_k(-))^2 + (landmark_y[i] - y_k(-))^2)

# Fx(h) = {y - landmark_y[i]} / {[}sqrt((landmark_x[i] - x_k(-))^2 + (landmark_y[i] - y_k(-))^2)}
def get_H():
    # e.g. L0[0]: X, L0[1]: Y
    # Following derivations are basically the same but they have different landmark chosen row wise
    H = np.array
    ([
        [(x_t_neg - L0[0]) / math.sqrt((L0[0] - x_t_neg)**2 + (L0[1] - y_t_neg)**2), (y_t_neg - L0[1]) / math.sqrt((L0[0] - x_t_neg)**2 + (L0[1] - y_t_neg)**2), 0],
        [(x_t_neg - L1[0]) / math.sqrt((L1[0] - x_t_neg)**2 + (L1[1] - y_t_neg)**2), (y_t_neg - L1[1]) / math.sqrt((L1[0] - x_t_neg)**2 + (L1[1] - y_t_neg)**2), 0],
        [(x_t_neg - L2[0]) / math.sqrt((L2[0] - x_t_neg)**2 + (L2[1] - y_t_neg)**2), (y_t_neg - L2[1]) / math.sqrt((L2[0] - x_t_neg)**2 + (L2[1] - y_t_neg)**2), 0]
    ])
    return H

def get_Q():
    # Get value of L and return L*M*L'
    L = get_L()
    # Diagonal 
    M = np.array
    ([
        [math.pow(delta_hat_rot_1 + np.random.normal(0, 0.2, 1), 2), 0, 0],
        [0, math.pow(delta_hat_trans + np.random.normal(0, 0.2, 1), 2), 0],
        [0, 0, math.pow(delta_hat_rot_2 + np.random.normal(0, 0.2, 1), 2)]
    ])
    # Q_k_1 = L_k_1 * M * L_k_1'
    return np.matmul(np.matmul(L, M), np.transpose(L))

# Just for initialization purposes
# Covariance matrix
P_k_neg = np.array
([
    [0,0,0],
    [0,0,0],
    [0,0,0]
])
P_k_pos = np.array
([
    [0,0,0],
    [0,0,0],
    [0,0,0]
])

# Kalman gain
K_k = np.array
([
    [0,0,0],
    [0,0,0],
    [0,0,0]
])

# constants
wheel_radius = 0.1
dist_bw_wheels = 0.4


def landmark_dist_callback(data):
    """
    This function handles updates when sensor measures are received
    """
    global P_k_pos, x_t1_pos
    # Distances got from sensor to landmarks
    LO_distance = data.position[0]
    L1_distance = data.position[1]
    L2_distance = data.position[2]
    z_k = np.array([LO_distance, L1_distance, L2_distance])

    # Covariance prediction: P_k(-) 
    P_k_neg = get_A() * P_k_pos * np.transpose(get_A()) + get_Q

    # Calculate Kalman gain

    K_k = P_k_neg * np.transpose(get_H) * np.linalg.inv(get_H() * P_k_neg * np.transpose(get_H))

    # z_hat_k = Hk * x_hat_k(-)
    z_hat_k =  np.matmul(get_H(), np.array([x_t1_pos, y_t1_pos, rz_t1_pos])) 

    # Posterior update
    # (I - K_k * H_k) * P_k(-)
    P_k_pos = np.matmul(np.eye(3) - np.matmul(K_k, get_H()), P_k_neg)
    diff_z = z_k - z_hat_k

    x_updated = np.array([[x_t1_pos, y_t1_pos, rz_t1_pos]]) + np.matmul(K_k, diff_z)

    # Store for next iteration as updated version
    # NOTE: There is bit stupid naming convention because no point in creating both P_k-1(+) and P_k-1(-)
    # Basically they are same but sometimes the prediction part (joint states callback function) has updated version of these
    P_k_neg = P_k_pos
    x_t1_pos = x_updated


def jointstate_callback(data):
    """
    This function handles predictions when joint_states are received
    """

    # Global definitions as these are changed during execution
    global prev_t, prev_wheel_pos, rz_t1_pos, x_t_storage, y_t_storage, \
        cumulative_t, x_t1_pos, y_t1_pos, start_time, t_cumulative_storage, x_t_storage, y_t_storage

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

    # Helper variables for predictions that have
    delta_rot_1 = delta_hat_rot_1 
    delta_trans = delta_hat_trans
    delta_rot_2 = delta_hat_rot_2

    # Calculate sample predictions x_t(-) and y_t(-)
    x_t = x_t1_pos + delta_trans * math.cos(rz_t1_pos + delta_rot_1)
    y_t = y_t1_pos + delta_trans * math.sin(rz_t1_pos + delta_rot_1)
    rz_t1_pos = rz_t1_pos + delta_rot_1 + delta_rot_2 # all particles rz_t_prev does not need to be stored for plotting
    quat_array = tf.transformations.quaternion_from_euler(0, 0, rz_t1_pos)

    pose_with_cov.header.stamp = data.header.stamp
    pose_with_cov.pose.pose.position.x = x_t
    pose_with_cov.pose.pose.position.y = x_t

    # Store for next iteration x_t-1(-) & y_t-1(-)
    x_t1_pos = x_t
    y_t1_pos = y_t

    # Add calculated values to storage for later use
    x_t_storage = np.array(np.column_stack([t_cumulative_storage, x_t])) \
            if x_t_storage.size else np.array([x_t]) 

    y_t_storage = np.array(np.column_stack([y_t_storage, y_t])) \
            if y_t_storage.size else np.array([y_t])

    t_cumulative_storage = np.array(np.column_stack([t_cumulative_storage, cumulative_t])) \
            if t_cumulative_storage.size else np.array([cumulative_t])

    

    # Broadcasting TF---------------------------------------------------
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "Odom"
    t.child_frame_id = "robot_base_kf"
    t.transform.translation.x = pose_with_cov.pose.pose.position.x
    t.transform.translation.y = pose_with_cov.pose.pose.position.x
    t.transform.translation.z = 0.0
    t.transform.rotation.x = quat_array[0]
    t.transform.rotation.y = quat_array[1]
    t.transform.rotation.z = quat_array[2]
    t.transform.rotation.w = quat_array[3]

    br.sendTransform(t)
    rospy.loginfo("Publishing odometry and TF.....................")

    # Update previous x and y for next iteration
    x_t1_pos = x_t
    y_t1_pos = y_t

    
def extended_kalman_filter():
    rospy.init_node('wheel_odometry', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, jointstate_callback)
    rospy.Subscriber("/Landmark_dist", JointState, landmark_dist_callback)
    rospy.loginfo("Extended Kalman Filtering has started")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def plot_result(case):
    """
    This function is used to plot the results of exercise
    """
 
    if case == 1:
        print(np.shape(x_t_storage))
    if case == 2:
        fig = plt.figure(case)
        ax1 = fig.add_subplot(111)

        # Get number of columns stored
        _, cols = np.shape(x_t_storage)
        # Generate as many colors as columns exist

        ax1.plot(t_cumulative_storage[0,:], y_t_storage[0,:], linewidth=1, alpha=0.2, c="b")
        ax1.plot(t_cumulative_storage[0,:], x_t_storage[0,:], linewidth=1, alpha=0.2, c="k")

if __name__ == '__main__':
    plot_result(1)
    extended_kalman_filter()
    print(f'program ended by user action: CTRL + C')
    plot_result(2)