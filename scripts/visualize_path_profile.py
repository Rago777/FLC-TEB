#!/usr/bin/env python

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and plots the current velocity.
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32
from nav_msgs.msg import Path
import numpy as np
import matplotlib.pyplot as plotter

def feedback_callback(data):
  global path

  if not data.poses: # empty
    path = []
    rospy.loginfo("no data")
    return
  path = data.poses
  
  
def plot_poses_profile(fig, path_plot, x, y):
  path_plot.cla()
  path_plot.grid()
  path_plot.set_xlabel('x [m]')
  path_plot.set_ylabel('y [m]')
  path_plot.plot(x, y, '-bx')
  fig.canvas.draw()

  
  
def path_plotter():
  global path
  rospy.init_node("visualize_path_profile", anonymous=True)
  
  topic_name = "/test_optim_node/local_plan"
  topic_name = rospy.get_param('~feedback_topic', topic_name)
  rospy.Subscriber(topic_name, Path, feedback_callback, queue_size = 100) # define feedback topic here!

  rospy.loginfo("Visualizing path profile published on '%s'.",topic_name) 
  # rospy.loginfo("Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")

  # two subplots sharing the same t axis
  fig, path_plot = plotter.subplots()
  plotter.ion()
  plotter.show()
  

  r = rospy.Rate(2) # define rate here
  while not rospy.is_shutdown():
    
    x = []
    y = []
    
    for point in path:
      x.append(point.pose.position.x)
      y.append(point.pose.position.y)
          
    plot_poses_profile(fig, path_plot, np.asarray(x), np.asarray(y))
        
    r.sleep()

if __name__ == '__main__': 
  try:
    path = []
    path_plotter()
  except rospy.ROSInterruptException:
    pass

