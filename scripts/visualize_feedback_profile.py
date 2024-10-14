#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plotter
import pandas as pd
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, TriggerResponse

# Global variables for trajectory and path
trajectory = []
path = []

# Callback function for velocity feedback
def velocity_feedback_callback(data):
    global trajectory
    if not data.trajectories:  # Empty data
        trajectory = []
        return
    trajectory = data.trajectories[data.selected_trajectory_idx].trajectory

# Callback function for path feedback
def path_feedback_callback(data):
    global path
    if not data.poses:  # Empty data
        path = []
        rospy.loginfo("No path data received")
        return
    path = data.poses

# Function to plot velocity and acceleration profiles
def plot_velocity_profile(ax_v, ax_omega, t, v, omega):
    ax_v.cla()
    ax_v.grid()
    ax_v.set_ylabel('Trans. velocity [m/s]')
    ax_v.plot(t, v, '-bx')
    
    ax_omega.cla()
    ax_omega.grid()
    ax_omega.set_ylabel('Rot. velocity [rad/s]')
    ax_omega.set_xlabel('Time [s]')
    ax_omega.plot(t, omega, '-bx')

# Function to plot acceleration profiles
def plot_acceleration_profile(ax_v, ax_omega, t, v, omega):
    ax_v.cla()
    ax_v.grid()
    ax_v.set_ylabel('Trans. acceleration [m/s^2]')
    ax_v.plot(t, v, '-bx')
    
    ax_omega.cla()
    ax_omega.grid()
    ax_omega.set_ylabel('Rot. acceleration [rad/s^2]')
    ax_omega.set_xlabel('Time [s]')
    ax_omega.plot(t, omega, '-bx')

# Function to plot the path profile
def plot_path_profile(ax, x, y):
    ax.cla()
    ax.grid()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.plot(x, y, '-bx')

# Service callback to export data to Excel
def export_to_excel(req):
    rospy.loginfo("Exporting data to Excel...")
    
    # Prepare data for velocity and acceleration
    t = []
    v = []
    omega = []
    linear_acceleration = []
    angular_acceleration = []

    for point in trajectory:
        t.append(point.time_from_start.to_sec())
        v.append(point.velocity.linear.x)
        omega.append(point.velocity.angular.z)

    # Calculate acceleration
    for i in range(1, len(v)):
        dt = t[i] - t[i-1]
        linear_acc = (v[i] - v[i-1]) / dt
        angular_acc = (omega[i] - omega[i-1]) / dt
        linear_acceleration.append(linear_acc)
        angular_acceleration.append(angular_acc)
    
    # Trim the last timestamp for acceleration data
    t_acc = t[:-1]

    # Prepare data for path
    x = []
    y = []
    for pose in path:
        x.append(pose.pose.position.x)
        y.append(pose.pose.position.y)

    # Create DataFrames for velocity, acceleration, and path
    velocity_data = pd.DataFrame({
        'Time [s]': t,
        'Trans. Velocity [m/s]': v,
        'Rot. Velocity [rad/s]': omega
    })
    
    acceleration_data = pd.DataFrame({
        'Time [s]': t_acc,
        'Trans. Acceleration [m/s^2]': linear_acceleration,
        'Rot. Acceleration [rad/s^2]': angular_acceleration
    })

    path_data = pd.DataFrame({
        'X [m]': x,
        'Y [m]': y
    })

    # Write to Excel
    with pd.ExcelWriter('trajectory_data.xlsx') as writer:
        velocity_data.to_excel(writer, sheet_name='Velocity', index=False)
        acceleration_data.to_excel(writer, sheet_name='Acceleration', index=False)
        path_data.to_excel(writer, sheet_name='Path', index=False)
    
    rospy.loginfo("Data exported successfully to trajectory_data.xlsx")
    return TriggerResponse(success=True, message="Data exported to Excel successfully.")

# Main plotter function
def combined_plotter():
    rospy.init_node("combined_visualizer", anonymous=True)

    # Subscribe to the topics
    velocity_topic = "/move_base/TebLocalPlannerROS/teb_feedback"
    path_topic = "/move_base/TebLocalPlannerROS/local_plan"
    
    rospy.Subscriber(velocity_topic, FeedbackMsg, velocity_feedback_callback, queue_size=1)
    rospy.Subscriber(path_topic, Path, path_feedback_callback, queue_size=100)

    rospy.loginfo("Subscribed to velocity and path topics.")

    # Create a ROS service to export data
    rospy.Service('export_to_excel', Trigger, export_to_excel)

    # Create subplots for velocity, acceleration, and path
    fig, ((ax_vel_v, ax_vel_omega), (ax_acc_v, ax_acc_omega), (ax_path, _)) = plotter.subplots(3, 2, figsize=(10, 12))
    plotter.ion()
    plotter.show()

    r = rospy.Rate(2)  # Set update rate to 2 Hz

    while not rospy.is_shutdown():
        # Velocity plot data
        t = []
        v = []
        omega = []
        linear_acceleration = []
        angular_acceleration = []

        for point in trajectory:
            t.append(point.time_from_start.to_sec())
            v.append(point.velocity.linear.x)
            omega.append(point.velocity.angular.z)

        # Calculate acceleration
        for i in range(1, len(v)):
            dt = t[i] - t[i-1]
            linear_acc = (v[i] - v[i-1]) / dt
            angular_acc = (omega[i] - omega[i-1]) / dt
            linear_acceleration.append(linear_acc)
            angular_acceleration.append(angular_acc)
        
        t_acc = t[:-1]  # Trim last timestamp for acceleration

        # Path plot data
        x = []
        y = []
        for pose in path:
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)

        # Update plots
        plot_velocity_profile(ax_vel_v, ax_vel_omega, np.asarray(t), np.asarray(v), np.asarray(omega))
        plot_acceleration_profile(ax_acc_v, ax_acc_omega, np.asarray(t_acc), np.asarray(linear_acceleration), np.asarray(angular_acceleration))
        plot_path_profile(ax_path, np.asarray(x), np.asarray(y))
        
        fig.canvas.draw()
        r.sleep()

if __name__ == '__main__':
    try:
        combined_plotter()
    except rospy.ROSInterruptException:
        pass
