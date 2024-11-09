#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import csv
from datetime import datetime
import tf.transformations  # For converting quaternions to Euler angles
import rosbag

# Initialize the ROS node (if needed)
rospy.init_node('odom_bag_reader', anonymous=True)

# Open a CSV file for writing
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = "odom_data_{}.csv".format(timestamp)
with open(filename, 'w') as file:
    writer = csv.writer(file)
    # Write the CSV header
    writer.writerow(['time', 'x', 'y', 'yaw', 'linear_velocity', 'angular_velocity'])

    def process_odometry(odometry_msg, current_time):
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

    # Specify the path to your bag file
    bag_file_path = '/media/rago/TECLAST/flcteb_2024-11-08-20-42-56.bag'

    # Open and read the bag file
    with rosbag.Bag(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/odom']):
            # Convert ROS time to seconds
            current_time = t.to_sec()
            # Process each odometry message
            process_odometry(msg, current_time)

print("Data extraction completed and saved to", filename)
