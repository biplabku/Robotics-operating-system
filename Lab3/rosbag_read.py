#!/usr/bin/env python
import rosbag
import numpy as np
import math as mth
import tf
import sys
import roslib
import rospy

from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


'''
global msg_obs
for topic, msg, t in bag.read_messages(topics=['Movements']):
    msg_obs = msg
    break
pass
'''


# a cell = [1,1,3]
def cellMidPos(x):
	a = 20*x[0]+10
	b = 20*x[1]+10
	c = 90*x[2]+45
	z = np.array([a,b,c])
	return z
def adjustAngle(x):
	if (x < -180):
		return x+360
	if (x > 180):
		return x-360
	else:
		return x


def twoCellMovement(a,b):
	x = cellMidPos(a)
	
	y = cellMidPos(b)
	
	translation = np.sqrt( (x[0]-y[0])**2 + (x[1]-y[1])**2 )
	linAngle = np.degrees(mth.atan2((y[1]-x[1]),(y[0]-x[0])))
	if (linAngle < 0):
		linAngle = linAngle + 360
	
	
	rot1 = linAngle - x[2]
	rot1 = adjustAngle(rot1)
	rot2 = y[2] - linAngle
	rot2 = adjustAngle(rot2)
	

	twoCellMov = np.array([rot1,translation,rot2])
	
	return twoCellMov

def MovementsMsgtoActualMovement_deg_transl_deg(msg):
	x = np.array([msg.rotation1.x,msg.rotation1.y,msg.rotation1.z,msg.rotation1.w])
	c = tf.transformations.euler_from_quaternion(x)
	rot1 = (c[2]*180)/mth.pi

	translation = msg.translation*100

	x = np.array([msg.rotation2.x,msg.rotation2.y,msg.rotation2.z,msg.rotation2.w])
	c = tf.transformations.euler_from_quaternion(x)
	rot2 = (c[2]*180)/mth.pi

	f = np.array([rot1,translation,rot2])
	return f

def gaussianPdf(x,mu,sig):
	a = sig*np.sqrt(2*mth.pi)
	b = -1.0*((x-mu)**2)
	c = 2*((sig)**2)
	return (1/a)*(np.exp(b/c))

def probOfInterCellMotion_GivMovMsg(a,b,msg):
	MovMsg = MovementsMsgtoActualMovement_deg_transl_deg(msg)
	TwoCellMov = twoCellMovement(a,b)
	PatA = location_probdis[a[0],a[1],a[2]]
	x = gaussianPdf(TwoCellMov[0],MovMsg[0],45)
	y = gaussianPdf(TwoCellMov[1],MovMsg[1],10)
	z = gaussianPdf(TwoCellMov[2],MovMsg[2],45)
	return PatA*x*y*z

def argMaxNumToCoord(x):
	i = (x/140)%35
	j = (x/4)%35
	k = x%4
	return np.array([i,j,k])

def GetLocationProbDis(msg):
	temp_probs = np.copy(location_probdis)
	for i in range(35):
		for j in range(35):
			for k in range(4):
				x = np.array([i,j,k])
				if (location_probdis[i,j,k] > 0.1):
					for a in range(35):
						for b in range(35):
							for c in range(4):
								y = np.array([a,b,c])
								temp_probs[a,b,c] = temp_probs[a,b,c]+probOfInterCellMotion_GivMovMsg(x,y,msg)
	temp_probs = temp_probs/np.sum(temp_probs)				
	return temp_probs
'''
location_probdis = GetLocationProbDis(msg_obs)
print np.argmax(location_probdis)
print argMaxNumToCoord(np.argmax(location_probdis))
'''
def CellOfTag(i):
	if (i == 0):
		return np.array([125,525])
	if (i == 1):
		return np.array([125,325])
	if (i == 2):
		return np.array([125,125])
	if (i == 3):
		return np.array([425,125])
	if (i == 4):
		return np.array([425,325])
	if (i == 5):
		return np.array([425,525])

def ProcessObs(msg_obs):
	tagCell = CellOfTag(msg_obs.tagNum)
	rangeToTag = msg_obs.range*100
	c = tf.transformations.euler_from_quaternion(np.array([msg_obs.bearing.x,msg_obs.bearing.y,msg_obs.bearing.z,msg_obs.bearing.w]))
	rotToTag = np.degrees(c[2])
	return tagCell,rangeToTag,rotToTag

def CellToTag(a,tagCell):
	x = cellMidPos(a)
	y = tagCell
	translation = np.sqrt( (x[0]-y[0])**2 + (x[1]-y[1])**2 )
	linAngle = np.degrees(mth.atan2((y[1]-x[1]),(y[0]-x[0])))
	if (linAngle < 0):
		linAngle = linAngle + 360
	rot1 = linAngle - x[2]
	rot1 = adjustAngle(rot1)
	return translation,rot1

def probofCell_GivObsMsg(a,msg_obs):
	tagCell,rangeToTag,rotToTag = ProcessObs(msg_obs)
	cellToTagrange,cellToTagRot = CellToTag(a,tagCell)

	PatA = location_probdis[a[0],a[1],a[2]]
	x = gaussianPdf(cellToTagRot,rotToTag,45)
	y = gaussianPdf(cellToTagrange,rangeToTag,10)
	return PatA*x*y

def GetLocationProbDis_Obs(msg_obs):
	temp_probs = np.copy(location_probdis)
	for i in range(35):
		for j in range(35):
			for k in range(4):
				x = np.array([i,j,k])
				temp_probs[i,j,k] = probofCell_GivObsMsg(x,msg_obs)
	temp_probs = temp_probs/np.sum(temp_probs)				
	return temp_probs

#location_probdis = GetLocationProbDis_Obs(msg_obs)
#print argMaxNumToCoord(np.argmax(location_probdis))

#x = np.array([11,27,2])
#print probofCell_GivObsMsg(x,msg_obs)

def mainfxn(set):
  #rospy.NodeHandle n;
  #The topic specified here is which the marker read the line information from.
  global marker_pub
  global line_strip
  p = Point();
  p.x = set[0]/100.0;
  p.y = set[1]/100.0;
  p.z = 0;
  line_strip.points.append(p);
    # The line list needs two points for each line
  
  while (marker_pub.get_num_connections() < 1):
  	print "No connection"
  marker_pub.publish(line_strip);
  r.sleep();
  pass

def cubePublish(id,x,y):
	global marker_pub2
	marker = Marker()
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = rospy.Time.now(); # replaced ine here
	marker.ns = "evader";
	marker.id = id;
	marker.type = Marker.CUBE;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.3;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	while (marker_pub2.get_num_connections() < 1):
		print "No connection"
	marker_pub2.publish(marker);



if __name__=="__main__":
    	
    
    rospy.init_node('evader')

    bag = rosbag.Bag(sys.argv[1])

    myPos =[11,27,2]
    global location_probdis
    location_probdis = np.zeros((35,35,4))
    location_probdis[11,27,2] = 1
    

    global marker_pub
    
    marker_pub = rospy.Publisher('/visualization_topic',Marker, queue_size=10)
    marker_pub2 = rospy.Publisher('/visualization_topic2',Marker, queue_size=10)

    r = rospy.Rate(30) # 30hz

    cubePublish(2,1.25,5.25)
    cubePublish(3,1.25,3.25)
    cubePublish(4,1.25,1.25)
    cubePublish(5,4.25,1.25)
    cubePublish(6,4.25,3.25)
    cubePublish(7,4.25,5.25)



    '''
    mainfxn2(np.array([1.25,5.25]))
    mainfxn2(np.array([1.25,3.25]))
    mainfxn2(np.array([1.25,1.25]))
    mainfxn2(np.array([4.25,1.25]))
    mainfxn2(np.array([4.25,3.25]))
    mainfxn2(np.array([4.25,5.25]))
    '''
    global line_strip
    line_strip = Marker();
    line_strip.header.frame_id = "/my_frame";
    line_strip.header.stamp = rospy.Time.now(); # replaced ine here
    line_strip.action = Marker.ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.ns = "evader";
    line_strip.id = 1;
    line_strip.type = Marker.LINE_STRIP;
    line_strip.scale.x = 0.1;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    for topic, msg, t in bag.read_messages(topics=['Movements','Observations']):
    	if (topic == 'Movements'):
			print "Movements message incoming"
			location_probdis = GetLocationProbDis(msg)
			h = argMaxNumToCoord(np.argmax(location_probdis))
			print h
			t = cellMidPos(h)
			print t
			mainfxn(t)
    	if (topic == 'Observations'):
			print "Observations message incoming"
			location_probdis = GetLocationProbDis_Obs(msg)
			h  = argMaxNumToCoord(np.argmax(location_probdis))
			print h
			t = cellMidPos(h)
			print t
			mainfxn(t)
		

	
bag.close()
			
			