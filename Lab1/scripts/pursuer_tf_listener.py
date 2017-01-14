#!/usr/bin/env python  
import rospy
import math
import tf
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 

if __name__ == '__main__':
    rospy.init_node("pursuer_tf_listener")

    listener = tf.TransformListener()
    turtle_vel = rospy.Publisher("robot_1/cmd_vel", geometry_msgs.msg.Twist,queue_size=1)
    #listener.waitForTransform("/robot_1", "/robot_0","/World1",rospy.Time(),rospy.Duration(4.0))
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            now=rospy.Time.now()
	    past=now-rospy.Duration(5.0)
	    listener.waitForTransformFull("/robot_1", now,"/robot_0",past,"/World1",now, rospy.Duration(1.0))
	    (trans,rot) = listener.lookupTransformFull("/robot_1",now, "/robot_0",past, "/World1")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
	rospy.loginfo("/%s", trans[0], trans[1])
        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()
