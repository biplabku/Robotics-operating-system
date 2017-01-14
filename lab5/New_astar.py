#!/usr/bin/env python
import numpy
from heapq import *
import rospy
import roslib
import math
import random
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


# input grid map provided from the text file
grid_map = numpy.array([[
	0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
      0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
      1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
      0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
      0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
      0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
      0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,
      0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
      0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
      0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
      0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
      0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
      0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
      0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
      0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
      0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
      0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
      0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]])

def heuristic(x, y):
    b=(y[0]-x[0])**2+(y[1]-x[1])**2
    return b


#astar help inputs from wikipedia
# working correctly
def astar(gmap, start, goal):
    openSet = []
    closeSet = set()
    cameFrom = {}
    chck_neighbor = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    gScore = {start:0}
    fScore = {start:heuristic(start, goal)}
    heappush(openSet, (fScore[start], start))
    while openSet:
    	current = heappop(openSet)[1]
    	if current == goal:
        	reconstructPath = []
        	while current in cameFrom:
            		reconstructPath.append(current)
            		current = cameFrom[current]
        	return reconstructPath(cameFrom,goal)
    	closeSet.add(current)
    	for i, j in chck_neighbor:
        	Adj_neigh = current[0] + i, current[1] + j            
        	tentativeGSscore = gScore[current] + heuristic(current, Adj_neigh)
        	if 0 <= Adj_neigh[0] < gmap.shape[0]:
            		if 0 <= Adj_neigh[1] < gmap.shape[1]:                
                		if gmap[Adj_neigh[0]][Adj_neigh[1]] == 1:
                    			continue
            		else:
                		continue
        	else:
			continue
        	if Adj_neigh in closeSet and tentativeGScore >= gScore.get(Adj_neigh, 0):
            		continue
        	if  tentativeGScore < gScore.get(Adj_neigh, 0) or Adj_neigh not in [i[1]for i in openSet]:
            		cameFrom[Adj_neigh] = current
            		gScore[Adj_neigh] = tentativeGScore
            		fScore[Adj_neigh] = tentativeGScore + heuristic(Adj_neigh, goal)
            		heappush(openSet, (fScore[neighbor], Adj_neigh))
            
    return False
    pub=rospy.publisher('cmd_vel',Twist,queue_size=10)
    twist = Twist()
    pub.publish(twist)
    rospy.loginfo("Curr goal distance " + str(goal_distance))

# working part correctly
def reconstructPath(cameFrom, current):
    totalPath = [current]
    while (current) in cameFrom:
        current = cameFrom[(current)]
        totalPath.append(current)
    return totalPath[::-1]


if __name__=="__main__":
    def __init__(self):
	self.msg = Twist()
	rospy.init_node('evader')
	self.pub = rospy.Publisher('/robot_0/cmd_vel', Twist,queue_size=100)
	rospy.Subscriber('/robot_0/base_scan', LaserScan, self.callback_laser)
	#yet to understand how to assign the cmd_vel values
    def Robot_goal(self,data):
	x = -8.0 - (data.pose.pose.position.y)
	y = -2.0 + (data.pose.pose.position.x)
	goal_distance = math.sqrt(math.pow((4.5 - x),2) + math.pow( (9-y) , 2 ))
	goal_angle = math.atan2((9-y),(4.5-x))
	fixed_goal_angle = math.atan2((9-(-2)),(4.5-(-8)))
	robot_angle = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    astar(grid_map, (-8.0,-2.0), (4.5,9))
