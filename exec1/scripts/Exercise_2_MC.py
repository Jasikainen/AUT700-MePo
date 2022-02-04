#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf, math
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt 

# Exercise 1. answer is used as an base for exercise 2. 2) 
# Dead-reckoning for differential drive model and Monte Carlo sampling

# All vectors are in column form which is as (n,) in numpy. x and y particle results
# are stacked column wise together -> columns of x and y matrices are then paired in graphs.

# Jani Asikainen
# Henri Pulkkinen

# NOTE: add exec1/Exercise_2_MC.py file inside CMakeLists.txt file of package

# 1. run roscore to get things working
# roscore

# 2. Startup the node
# rosrun exec1 Exercise_2_MC.py

# 3. Start recording odometry messages
# rosbag record -o odometry_ex2 odometry

# 4. play the Exercise-2.bag file that is listened to by ROS node
# rosbag play ~/Exercise-2.bag

# -------------------------------------


"""
NOTE: NO NEED FOR PUBLISHING ODOMETRY HERE. CAN NOT BE DONE AS THERE IS 100 PARTICLES

pub = rospy.Publisher('odometry', Odometry, queue_size=10)
odom = Odometry()
odom.header.frame_id = "Odom"
odom.child_frame_id = "robot_base"
"""

prev_t = 0
prev_wheel_pos = [0, 0]

# Perform simulation of these 100 particles
# subscript: t-1
draw_samples = 100
x_t_prev = np.zeros(draw_samples,) # (100,)
y_t_prev = np.zeros(draw_samples,)
rz_t_prev = np.zeros(draw_samples,) # All particles have zero as their initial state...

# Preallocate 100 empty that are used to store new states in calculation
# subscript: t
x_t = np.empty(draw_samples,)
y_t = np.empty(draw_samples,)

cumulative_t = 0.0 # keeps track of cumulative time. Determines when to store states of particles.
start_time = 0
seconds_between = 3 # Change this to change storage time e.g. 2 s: 0, 2, 4, 6, etc...
x_t_storage = np.array([]) # initialize empty for storaging states of particles
y_t_storage = np.array([])
rz_t_avg_storage = np.array([]) # THIS IS USED TO STORE ONLY AVERAGES AS RADIANS. ONLY NEEDED FOR PLOTTING PURPOSES.
x_t_avg_storage = np.array([]) # ONLY NEEDED FOR PLOTTING PURPOSES.
y_t_avg_storage = np.array([]) # ONLY NEEDED FOR PLOTTING PURPOSES.

# constants
wheel_radius = 0.1
dist_bw_wheels = 0.4
a1, a2, a3, a4 = 50, 5, 70, 10


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

    # Store every n seconds between starting from zero. 
    # Matching example, when seconds_between 3: 
    # 3.0023123 * 100 -> 300.2f -> 300i % (3*100) -> True
    if int((cumulative_t - start_time) * 100) % (seconds_between * 100) == 0:
        # Prints here may be used to debug program if needed
        rospy.loginfo(f"----------------------")
        rospy.loginfo(f"cumulative_t : {cumulative_t - start_time:.3f}")
        rospy.loginfo(f"average x_t : {np.average(x_t):.3f} y_t : {np.average(y_t):.3f}")
        rospy.loginfo(f"angle : {np.average(rz_t_prev * 180 / math.pi):.3f}")

        # On first iteration just assign equal to x_t and y_t in future stack columns
        x_t_storage = np.array(np.column_stack([x_t_storage, x_t])) \
            if x_t_storage.size else x_t

        y_t_storage = np.array(np.column_stack([y_t_storage, y_t])) \
            if y_t_storage.size else y_t

        rz_t_avg_storage = np.array(np.column_stack([rz_t_avg_storage, np.average(rz_t_prev)])) \
            if rz_t_avg_storage.size else np.average(rz_t_prev) # scalar values only here
    
    # Store averages of each step for plotting average placement at time stamps
    x_t_avg_storage = np.array(np.column_stack([x_t_avg_storage, np.average(x_t)])) \
            if x_t_avg_storage.size else np.average(x_t) # scalar values only here
    y_t_avg_storage = np.array(np.column_stack([y_t_avg_storage, np.average(y_t)])) \
            if y_t_avg_storage.size else np.average(y_t) # scalar values only here

    # Update previous x and y for next iteration
    x_t_prev = x_t
    y_t_prev = y_t

    
def wheel_odometry():
    rospy.init_node('wheel_odometry', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.loginfo("Monte carlo particle samping started")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def plot_result(case):
    """
    This function is used to plot the results of exercise
    """
    if case == 1:
        print(np.shape(x_t_storage))
        
    elif case == 2:
        fig = plt.figure(case)
        ax1 = fig.add_subplot(111)

        # Get number of columns stored
        _, cols = np.shape(x_t_storage)
        # Generate as many colors as columns exist
        color = plt.cm.rainbow(np.linspace(0, 1, cols))
        length_arrow = 10
        for idx, c in zip(range(cols), color):
            label = f"{(idx)*seconds_between} s"
            label_avg = f"avg@{(idx)*seconds_between} s"

            avg_x = np.average(x_t_storage[:,idx])
            avg_y = np.average(y_t_storage[:,idx])

            # Calculate coordinates where arrow points at to plot quiver
            avg_rotz = rz_t_avg_storage[:,idx]
            avg_rotz_deg = float(avg_rotz) * 180.0 / math.pi
            dir_x = math.cos(avg_rotz)
            dir_y = math.sin(avg_rotz)

            # Text placement depends on the avg angle of particles
            if avg_rotz_deg < 90:
                ax1.text(avg_x-0.1, avg_y+0.1, '{:.2f}$^\circ$'.format(avg_rotz_deg), \
                    style='italic', fontsize=10)
            else:
                ax1.text(avg_x+0.05, avg_y+0.1, '{:.2f}$^\circ$'.format(avg_rotz_deg), \
                    style='italic', fontsize=10)

            ax1.scatter(avg_x, avg_y, s=400, c=c, linewidth=3, marker="o", alpha=0.3)
            ax1.quiver(avg_x, avg_y, dir_x, dir_y, width=0.005)
            ax1.scatter(x_t_storage[:,idx], y_t_storage[:,idx], s=4, c=c, marker="o", label=label)
        
        x_t_avg_storage
        ax1.plot(x_t_avg_storage[0,:], y_t_avg_storage[0,:], linewidth=1, alpha=0.2, c="k")

        ax1.legend(bbox_to_anchor=(0., 1.08, 1., .102), loc='lower left', fontsize=10,
                      ncol=6, mode="expand", borderaxespad=0.)
        ax1.set_title('Simulation {} seconds between particle states'.format(seconds_between))
        ax1.set_xlabel("x (meters)")
        ax1.set_ylabel("y (meters)")
        plt.savefig('Exercise_2_Monte_carlo.png', dpi=300, bbox_inches="tight")
        plt.show()


if __name__ == '__main__':
    plot_result(1) # Draw particles initially
    wheel_odometry()
    print(f'program ended by user action: CTRL + C')
    print(np.shape(x_t_storage))
    print(np.shape(y_t_storage))
    plot_result(2)