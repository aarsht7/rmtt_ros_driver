#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, Twist, PointStamped
from sensor_msgs.msg import Joy
import numpy as np


# Global variables 
global z_ref, kp, ki, integral
z_ref = 1.0
kp = 0.6
ki = 0.0
integral = 0.0
integralX = 0.0
integralY = 0.0
integralZ = 0.0
buttons = None
automatic_control_enabled = False
mocap_position = np.zeros((3,1))	# position vectors 




command = Twist()

x = 0.0
y = 0.0
z = 0.0


# Node init
rospy.init_node('cmd', anonymous=False)

# reference
# can also be coded as global variable, input topic, value read from textfile, etc.
x_ref = rospy.get_param("~xref",1.) # m
y_ref = rospy.get_param("~yref",1.) # m
z_ref = rospy.get_param("~zref",1.) # m
x_w = [-1,1,-1]
y_w = [1,-1,-1]
i = 0
freq = 10
Ts = 1.0/freq

# Publisher declaration on the topic of command
pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
commandRate = rospy.Rate(freq) 

pubDerivative = rospy.Publisher('/vz', Float32, queue_size=10)
pubFilteredDerivative = rospy.Publisher('/vz_filt', Float32, queue_size=10)
pubIntegral = rospy.Publisher('/integral', Float32, queue_size=10)


# Measurements reading
def readAltitude(msg):
    global z
    z = msg.data
    #rospy.loginfo(" z (m) %f ", z)
    
# Ref altitude reading
def readZref(msg):
    global z_ref
    z_ref = msg.data
    
# Ref altitude reading
def readXref(msg):
    global x_ref
    x_ref = msg.data
    
# Ref altitude reading
def readYref(msg):
    global y_ref
    y_ref = msg.data
    
    

# -----------------------------------------------------------------------------
def callBackPose(data):
# -----------------------------------------------------------------------------
    global mocap_position
         
    mocap_position[0,0] = data.point.x  
    mocap_position[1,0] = data.point.y
    mocap_position[2,0] = data.point.z
    
    
# -----------------------------------------------------------------------------



# -----------------------------------------------------------------------------
def callBackJoystick(data):
# -----------------------------------------------------------------------------
	global buttons, automatic_control_enabled
	
	for i in range(0, len(data.buttons)):
		if buttons == None or data.buttons[i] != buttons[i]:
			'''
			if i == 0 and data.buttons[i] == 1: # and flag_land != None:
				mas_control_enabled = False
			if i == 1 and data.buttons[i] == 1:  ##emergency
				mas_control_enabled = False
			if i == 2 and data.buttons[i] == 1: # and flag_takeoff != None:
				mas_control_enabled = False
			'''
			if i == 5 and data.buttons[i] == 1:
				automatic_control_enabled = (not automatic_control_enabled)
				
				if automatic_control_enabled:
					rospy.loginfo("Automatic control ON")
				else:
					rospy.loginfo("Automatic control OFF")					
	buttons = data.buttons
# -----------------------------------------------------------------------------




    
'''
# proportional gain
def readKp(msg):
    global kp
    kp = msg.data
    
# integral gain
def readKi(msg):
    global ki
    ki = msg.data
'''    

# Subscriber declaration 
rospy.Subscriber("/btm_range", Float32, readAltitude)
rospy.Subscriber("/x_ref", Float32, readXref)
rospy.Subscriber("/y_ref", Float32, readYref)
rospy.Subscriber("/z_ref", Float32, readZref)
rospy.Subscriber("/natnet_ros/rmtt1/position", PointStamped, callBackPose)
#rospy.Subscriber("/kp", Float32, readKp)
#rospy.Subscriber("/ki", Float32, readKi)


rospy.Subscriber("/joy", Joy, callBackJoystick)

# Main: looping execution
if __name__ == '__main__':

    epsilon_prec = 0.0
    filtered_derivative = 0.0
    
    
    
    while not rospy.is_shutdown():
        #rospy.loginfo(" zref=%f m,  z=%f m", z_ref, z)

        
        
        x = mocap_position[0,0]
        y = mocap_position[1,0]
        z = mocap_position[2,0]
        print("     xref={0} m,  x={1} m".format(x_ref, x))
        print("     yref={0} m,  y={1} m".format(y_ref, y))
        print("     zref={0} m,  z={1} m".format(z_ref, z))

        epsilonX = x-x_ref
        epsilonY = y-y_ref
        epsilonZ = z-z_ref

        #if epsilonX<0.1 and epsilonY<0.1 and i<len(x_w):
        #    x_ref = x_w[i]
        #    y_ref = y_w[i]
        #    i+=1
        
        
        integralX = integralX + Ts*epsilonX
        integralX = integralX + Ts*epsilonX
        integralX = integralX + Ts*epsilonX
        
        #pubIntegral.publish(Float32(integral))
        
        #derivative = (epsilon - epsilon_prec)/Ts
        
        #alpha = 0.7

        #filtered_derivative = alpha*filtered_derivative + (1.0-alpha)*derivative 

        #pubDerivative.publish(Float32(derivative))
        #pubFilteredDerivative.publish(Float32(filtered_derivative))


        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        
        if (automatic_control_enabled):
        	command.linear.x = -0.8*(epsilonX) #- 0.1*integralX       # ok parfait
        	command.linear.y = -0.8*(epsilonY) #- 0.1*integralY       # ok parfait
        #command.linear.z = -0.6*(epsilon) - 0*integral     # lent et erreur statique
        #command.linear.z = -1.2*(epsilon) - 0*integral     # erreur statique
        #command.linear.z = -0.2*(epsilon) - 0.3*integral        # oscillant
        #command.linear.z = -0.8*(epsilon) - 0.3*integral       # ok lent
        	command.linear.z = -1.2*(epsilonZ) - 0.1*integralZ       # ok parfait
        
        
        #command.linear.z = -kp*(epsilon) - ki*integral

        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
    
        # Command sending
        pubCommand.publish(command)
        commandRate.sleep()
        
        #epsilon_prec = epsilon




