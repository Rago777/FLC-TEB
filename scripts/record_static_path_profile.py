#!/usr/bin/env python

# This script captures the latest path data at startup, including orientation, and saves it to a file.
# Author: christoph.roesmann@tu-dortmund.de (modified by AI)

import rospy
from nav_msgs.msg import Path
import csv
import codecs
import tf.transformations  # Import transformations for quaternion to euler conversion

path = []

def feedback_callback(data):
    global path

    if not data.poses:  # empty
        rospy.loginfo("Received empty path data, skipping...")
        return

    path = data.poses
    rospy.loginfo("Path captured, saving to file...")

    with codecs.open('path_data_1.csv', 'w', encoding='utf-8') as csvfile:
        fieldnames = ['x', 'y', 'yaw']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for point in path:
            # Extract the quaternion
            q = (
                point.pose.orientation.x,
                point.pose.orientation.y,
                point.pose.orientation.z,
                point.pose.orientation.w
            )
            # Convert quaternion to euler angles (roll, pitch, yaw)
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
            
            writer.writerow({
                'x': str(point.pose.position.x),
                'y': str(point.pose.position.y),
                'yaw': str(yaw)
            })
    
    rospy.loginfo("Path saved to file.")
    rospy.signal_shutdown("Path saved, shutting down node.")

def capture_and_save_path():
    global path
    rospy.init_node("capture_path_profile", anonymous=True)
    
    topic_name = "/move_base/TebLocalPlannerROS/local_plan"
    topic_name = rospy.get_param('~feedback_topic', topic_name)
    rospy.Subscriber(topic_name, Path, feedback_callback, queue_size=100)  # define feedback topic here!

    rospy.loginfo("Capturing path profile published on '%s'.", topic_name)
    
    # Wait for the first message to be received
    while not path:
        if rospy.is_shutdown():
            return
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        path = []
        capture_and_save_path()
    except rospy.ROSInterruptException:
        pass