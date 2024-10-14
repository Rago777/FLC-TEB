#!/usr/bin/env python

import rospy
from teb_local_planner.msg import FeedbackMsg
import csv
import codecs
import numpy as np

def feedback_callback(data):
    trajectories = data.trajectories
    rospy.loginfo("Received %d trajectories, saving each to a separate file...", len(trajectories))

    for idx, trajectory in enumerate(trajectories):
        file_name = 'path_data_{}.csv'.format(idx)
        rospy.loginfo("Saving trajectory %d to %s", idx, file_name)

        with codecs.open(file_name, 'w', encoding='utf-8') as csvfile:
            fieldnames = ['time', 'x', 'y', 'linear_velocity', 'angular_velocity']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            # Initialize lists to hold data for time, velocities
            time_data = []
            linear_velocity_data = []
            angular_velocity_data = []

            for point in trajectory.trajectory:
                # Append time and pose data
                time_data.append(point.time_from_start.to_sec())
                writer.writerow({
                    'time': point.time_from_start.to_sec(),
                    'x': point.pose.position.x,
                    'y': point.pose.position.y,
                    'linear_velocity': point.velocity.linear.x,
                    'angular_velocity': point.velocity.angular.z
                })

                # Collect velocities
                linear_velocity_data.append(point.velocity.linear.x)
                angular_velocity_data.append(point.velocity.angular.z)

            # Calculate time differences and velocities
            time_diff = np.diff(time_data, prepend=0)  # Prepend 0 for the first point
            avg_linear_velocity = np.mean(linear_velocity_data)
            avg_angular_velocity = np.mean(angular_velocity_data)

            rospy.loginfo("Average linear velocity: %f m/s", avg_linear_velocity)
            rospy.loginfo("Average angular velocity: %f rad/s", avg_angular_velocity)

    rospy.loginfo("All trajectories saved to separate files.")
    rospy.signal_shutdown("All paths saved, shutting down node.")

def capture_and_save_path():
    rospy.init_node("capture_path_profile", anonymous=True)

    topic_name = "/move_base/TebLocalPlannerROS/teb_feedback"
    topic_name = rospy.get_param('~feedback_topic', topic_name)
    rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback, queue_size=100)

    rospy.loginfo("Capturing trajectories published on '%s'.", topic_name)

if __name__ == '__main__':
    try:
        trajectory = []
        capture_and_save_path()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
