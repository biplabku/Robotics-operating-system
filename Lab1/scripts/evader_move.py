#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
def callback(data):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
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
    #talker(b)

def talker(a):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    	vel = Twist()
	vel.linear.x=2.0
	vel.linear.y=0.0
	vel.linear.z=0.0
	vel.angular.z=a	
	vel.angular.x=0.0
	vel.angular.y=0.0
	pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
	rospy.init_node('evader_move', anonymous=True)
	rospy.Subscriber("base_scan", LaserScan, callback)
	rospy.spin()	
    except rospy.ROSInterruptException:
        pass
