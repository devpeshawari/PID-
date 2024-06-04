#!/usr/bin/env python3

from operator import index
import rospy
import math
from sensor_msgs.msg import LaserScan
from f1tenth_simulator.msg import pid_input 


angle_range = 360	
forward_projection = 1.5		
desired_distance = 0.5
vel = 15 		
error = 0.0		
car_length = 0.50 


pub = rospy.Publisher('error', pid_input, queue_size=10)




def getRange(data,angle):
	
    
	index = angle * (len(data.ranges)/360)
	return data.ranges[int(index)]



def callback(data):
	global forward_projection
	global desired_distance 
	global error
	
	print(desired_distance)
	theta = 330
	a = getRange(data,theta) # obtain the ray distance for theta
	b = getRange(data, 270) 
	# print(b, '90 degree')
	theta = math.radians(theta)
	alpha = math.atan((((a * (math.cos(theta))) - b) / (a * (math.sin(theta)))))
	ab =  b * math.cos(alpha) 
	cd = ab + (forward_projection * math.sin(alpha))
	error = desired_distance - cd	
	msg = pid_input()	
	msg.pid_error = error	
	pub.publish(msg)
	
	
			


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_find',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/scan",LaserScan,callback)
	rospy.spin()