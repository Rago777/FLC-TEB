#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from teb_local_planner.msg import FeedbackMsg,TrajectoryMsg,TrajectoryPointMsg
from std_msgs.msg import String
import os

def feedback_callback(data):
  global trajectory

  # 如果轨迹为空，则将其设置为空，并返回
  if not data.trajectories: # empty
    trajectory = []
    return
  # 获取选定轨迹的信息
  trajectory = data.trajectories[data.selected_trajectory_idx].trajectory

  # 将轨迹信息写入txt文件
  with open("trajectory_data.txt", "a") as file:
    file.write("Timestamp: {}\n".format(rospy.Time.now()))
    for point in trajectory:
      file.write(
        "Time: {} Velocity: {} Omega: {} Acceleration: {} Omegadot: {}\n".format(
          point.time_from_start.to_sec(), 
          point.velocity.linear.x,
          point.velocity.angular.z,
          point.acceleration.linear.x,
          point.acceleration.angular.z)
            )

def velocity_print():
  global trajectory

  rospy.init_node("sub_print_velocity", anonymous=False)

  topic_name = "/move_base/TebLocalPlannerROS/teb_feedback"
  rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback, queue_size=1)

  # 如果文件存在，删除它
  if os.path.exists("trajectory_data.txt"):
    os.remove("trajectory_data.txt")

  rospy.spin()

if __name__ == "__main__":
  velocity_print()
