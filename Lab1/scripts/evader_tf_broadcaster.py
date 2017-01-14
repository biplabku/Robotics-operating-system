#!/usr/bin/env python  
import roslib
#roslib.load_manifest('lab1')
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry  
import tf


def callback(data):
    pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
    r_len=len(data.ranges)
    b=2
    a =0
    for i in range(0,r_len):
	if data.ranges[i]<0.69:
           b = 0
           a = 90	    
           break
    vel1= Twist()
    vel1.linear.x=b
    vel1.linear.y=0.0
    vel1.linear.z=0.0
    vel1.angular.z=a	
    vel1.angular.x=0.0
    vel1.angular.y=0.0
    pub.publish(vel1) 		

def handle_robot_0_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (0, 0, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "/robot_0/odom",
                     "World1")
def handle_robot_1_pose(msg):
   br1 = tf.TransformBroadcaster()
   br1.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (0, 0, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "/robot_1/odom",
                     "World1")
if __name__ == '__main__':
    rospy.init_node('evader_tf_broadcaster')
    rospy.Subscriber("/robot_0/base_scan", LaserScan, callback)
	
    rospy.Subscriber("/robot_0/odom",
                     Odometry,
                     handle_robot_0_pose)
    rospy.Subscriber("/robot_1/odom",
                     Odometry,
                     handle_robot_1_pose)
    
    rospy.spin()
