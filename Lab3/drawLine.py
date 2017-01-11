#!/usr/bin/env python
import roslib
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
#first you should create a package which depends on roscpp and visualization_msgs
#Then make required changes to CMakeList.txt
#When running rviz, don't forget to Add Marker
#And adjust the fixed frame to my_frame
def mainfxn(set):
  #rospy.NodeHandle n;
  #The topic specified here is which the marker read the line information from.
  marker_pub = rospy.Publisher('/visualization_topic',Marker, queue_size=10)
  r = rospy.Rate(30) # 30hz


  line_strip = Marker();
  #this frame here specifies the frame that the positions of your line points are with respect to that.
  line_strip.header.frame_id = "/my_frame";
  line_strip.header.stamp = rospy.Time.now(); # replaced ine here
  line_strip.action = Marker.ADD;
  line_strip.pose.orientation.w = 1.0;
  #The following positions are like applying a translation to all the points of the lines, For exmaple if you have (0,0) point in your line, it will be (1,1)
  #line_strip.pose.position.x = 1.0;
  #line_strip.pose.position.y = 1.0;
  #These two values identify a unique marker, if another marker with these values show up, this one is replaced
  line_strip.ns = "evader";
  line_strip.id = 1;
  #the life_time variable specifies the time length which your line is shown in rviz, after that it disappears. 
  #line_strip.lifetime = ros::Duration(0.5);
  line_strip.type = Marker.LINE_LIST;
  # LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  # Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0; # the alpha value shoud always be higger than zero
  #Here we wanna draw y=x line, so we should derive some points of that line and add them to line_strip
  #a = np.array(([1,100]))
  for i in range(len(set)):
    for j in range(2):
      p = Point();
      p.x = set[i][j,0];
      p.y = set[i][j,1];
      p.z = 0;
      line_strip.points.append(p);
    # The line list needs two points for each line
  pass
  marker_pub.publish(line_strip);
  r.sleep();
  pass


