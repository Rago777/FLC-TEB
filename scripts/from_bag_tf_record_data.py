#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import csv
from datetime import datetime
import tf.transformations  # For converting quaternions to Euler angles
import rosbag
import tf
import numpy as np

# Initialize the ROS node (if needed)
rospy.init_node('odom_bag_reader', anonymous=True)

# Open a CSV file for writing
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = "odom_data_{}.csv".format(timestamp)
with open(filename, 'w') as file:
    writer = csv.writer(file)
    # Write the CSV header
    writer.writerow(['time', 'x_corrected', 'y_corrected', 'yaw_corrected', 'linear_velocity', 'angular_velocity'])

    def transform_pose(odom_pose, tf_transform):
        """Applies a transform to a pose to correct it according to the map to odom transform."""
        # Extract the position from the odom pose
        position = [odom_pose.position.x, odom_pose.position.y, odom_pose.position.z]
        orientation = [
            odom_pose.orientation.x,
            odom_pose.orientation.y,
            odom_pose.orientation.z,
            odom_pose.orientation.w
        ]

        # Apply the transform
        transformed_position = tf.transformations.translation_matrix((tf_transform.translation.x, tf_transform.translation.y, tf_transform.translation.z))
        transformed_orientation = tf.transformations.quaternion_matrix((
            tf_transform.rotation.x,
            tf_transform.rotation.y,
            tf_transform.rotation.z,
            tf_transform.rotation.w
        ))
        odom_matrix = np.dot(tf.transformations.translation_matrix(position), tf.transformations.quaternion_matrix(orientation))
        corrected_matrix = np.dot(np.dot(transformed_position, transformed_orientation), odom_matrix)

        # Extract corrected position and orientation
        corrected_position = tf.transformations.translation_from_matrix(corrected_matrix)
        corrected_orientation = tf.transformations.quaternion_from_matrix(corrected_matrix)
        corrected_euler = tf.transformations.euler_from_quaternion(corrected_orientation)

        return corrected_position, corrected_euler[2]  # Return position and yaw

    def process_odometry(odometry_msg, current_time, map_to_odom_transform):
        # Extract linear and angular velocity data
        linear_x_speed = odometry_msg.twist.twist.linear.x
        angular_z_speed = odometry_msg.twist.twist.angular.z

        # Correct the pose using the transform
        corrected_position, corrected_yaw = transform_pose(odometry_msg.pose.pose, map_to_odom_transform)

        # Write the corrected data to the CSV file
        writer.writerow([current_time, corrected_position[0], corrected_position[1], corrected_yaw, linear_x_speed, angular_z_speed])

    # Specify the path to your bag file
    bag_file_path = '/media/rago/TECLAST/flcteb_2024-11-08-20-58-23.bag'

    # Open and read the bag file
    with rosbag.Bag(bag_file_path, 'r') as bag:
        map_to_odom_transform = None
        for topic, msg, t in bag.read_messages():
            if topic == "/tf" or topic == "/tf_static":
                for transform in msg.transforms:
                    if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                        map_to_odom_transform = transform.transform
            
            if topic == '/odom' and map_to_odom_transform is not None:
                # Convert ROS time to seconds
                current_time = t.to_sec()
                # Process each odometry message
                process_odometry(msg, current_time, map_to_odom_transform)

print("Data extraction completed and saved to", filename)
rospy.signal_shutdown("Data processing completed")
