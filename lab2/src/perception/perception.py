#!/usr/bin/env python

import rospy
import tf
import roslib
import math
import random
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

def callback(data):
    data_range=data.ranges # getting the data values 
    points=data_range	
    iterations= 10  	     # number of iterations
    threshold= 1    	     # threshold      # ratio of inliers required to assert
                             # that a model fits well to data
    for iter in range(iterations):
        idx1=random.randint(point_index)	# randomizing the indices
        sample=points[idx1,2]			# selecting 2 random points
        inlier=[]
        outlier=[]

	# creating a line model/ using the 2 points
	m=(sample[1].y-sample[0].y)/(sample[1].x-sample[0].x)  # slope(m) of the line
    	c=sample[1].y-m*sample[1].x                          # y-intercept of the line
	
 	# calculating the inlier and outlier
        for i in range(sample.shape[0]):
            point_x = sample[i].x
            point_y = sample[i].y
	    x1=(point_x+m*point_y-m*c)/(1+math.pow(m,2))		
    	    y1=(m*point_x+math.pow(m,2)*point_y-math.pow(m,2)*c)/(1+math.pow(m,2)+c)
	    # calulating the distance from the point to the line
            distFrmPoint = math.sqrt(math.pow((x1-point_x),2)+math.pow((y1-point_y),2))

            # checking the distance with respect to threshold
            if distFrmPoint<threshold:
                inlier.append(x)
	    else:
                outlier.append(y)

class percept:
    def __init__(self):
        while not rospy.is_shutdown():
                  rospy.Subscriber("robot_0/base_scan", LaserScan,callback)


if __name__=="__main__":
    rospy.init_node('perception')
    percept()
