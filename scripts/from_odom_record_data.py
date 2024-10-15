#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import csv
from datetime import datetime
import tf.transformations  # For converting quaternions to Euler angles

# Initialize the ROS node
rospy.init_node('odom_recorder')

# Open a CSV file for writing
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = "odom_data_{}.csv".format(timestamp)
with open(filename, 'wb') as file:
    writer = csv.writer(file)
    # Write the CSV header
    writer.writerow(['time', 'x', 'y', 'yaw', 'linear_velocity', 'angular_velocity'])

    def callback(odometry_msg):
        # Get the current time
        current_time = rospy.Time.now().to_sec()
        
        # Extract position data from the Odometry message
        x_position = odometry_msg.pose.pose.position.x
        y_position = odometry_msg.pose.pose.position.y
        
        # Convert the orientation from quaternion to Euler angles (roll, pitch, yaw)
        quaternion = (
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]  # Yaw is the third element in the returned tuple (x=roll, y=pitch, z=yaw)
        
        # Extract linear and angular velocity data
        linear_x_speed = odometry_msg.twist.twist.linear.x
        angular_z_speed = odometry_msg.twist.twist.angular.z
        
        # Write the data to the CSV file
        writer.writerow([current_time, x_position, y_position, yaw, linear_x_speed, angular_z_speed])

    # Create a subscriber to the /odom topic
    rospy.Subscriber('/odom', Odometry, callback)

    # Wait for callbacks
    rospy.spin()