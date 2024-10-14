#!/usr/bin/env python

import rospy
from teb_local_planner.msg import FeedbackMsg
import csv
import codecs

def feedback_callback(data):
    trajectories = data.trajectories
    rospy.loginfo("Received %d trajectories, saving each to a separate file...", len(trajectories))

    for idx, trajectory in enumerate(trajectories):
        file_name = 'path_data_trajectory_{}.csv'.format(idx)
        rospy.loginfo("Saving trajectory %d to %s", idx, file_name)
        
        with codecs.open(file_name, 'w', encoding='utf-8') as csvfile:
            fieldnames = ['x', 'y']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            for point in trajectory.trajectory:
                writer.writerow({
                    'x': point.pose.position.x,
                    'y': point.pose.position.y
                })

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
        capture_and_save_path()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
