#!/usr/bin/env python3
import math
import rospy
from f1tenth_simulator.msg import pid_input
from ackermann_msgs.msg import AckermannDriveStamped


pid = 0.00
kp = 1 #TODO
kd = 0.75 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	
prev_error = 0.0
vel_input = 2	


angle = 0.0

command_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 1)

def control(data):
	global pid
	global prev_error
	global vel_input
	global kp
	global kd
	global angle
	

	error = data.pid_error
	
	print((float(error) - float(prev_error)))
	pid = (float(kp) * float(error)) + (float(kd) * (float(error) - float(prev_error)))  
	prev_error = error
	
	
	
	angle = -pid
	

	command = AckermannDriveStamped()
	
	if command.drive.steering_angle <= 100 and angle >= -100: 
		command.drive.steering_angle = angle
		
    
	command.drive.speed = float(vel_input)

		
	command_pub.publish(command)
	

if __name__ == '__main__':
	
	rospy.init_node('control', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()