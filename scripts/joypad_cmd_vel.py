#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
   Sylvain BERTRAND, 2022

   Publish Twist from joypad inputs
   (all variables in SI unit)

'''

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from std_srvs.srv import Trigger



# node init
# --------------
rospy.init_node('joypad_cmd_vel', anonymous=False)



# parameters
# ------------
Vmax = rospy.get_param('~Vmax',0.20) # maximum speed (m/s)
Vmin = -Vmax
OmegaMax = rospy.get_param('~OmegaMax',1.57) # maximum angular speed (rad/s)
OmegaMin = -OmegaMax


# global variables
# -----------------
buttons = None
cmdVelMsg = Twist()


# control frequency
# -----------------
frequency = 10.0
Ts = 1.0/frequency
cmdPubRate = rospy.Rate(frequency)
ns = rospy.get_namespace().replace('/','')

# publishers
# ----------------
pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pubTakeOff = rospy.Publisher('takeoff', Empty, queue_size=10)
pubLand = rospy.Publisher('land', Empty, queue_size=10)
rospy.wait_for_service('/'+ns+"/emergency")
rospy.loginfo("found emergency service rmtt")
emergency = rospy.ServiceProxy('/'+ns+"/emergency", Trigger)

rospy.wait_for_service('/'+ns+"/takeoff")
rospy.loginfo("found takeoff service rmtt")
takeoff = rospy.ServiceProxy('/'+ns+"/takeoff", Trigger)

rospy.wait_for_service('/'+ns+"/land")
rospy.loginfo("found land service rmtt")
land = rospy.ServiceProxy('/'+ns+"/land", Trigger)

# -----------------------------------------------------------------------------
def callBackJoy(data):
# -----------------------------------------------------------------------------
    global cmdVelMsg, buttons

    vx = data.axes[4]
    vy = data.axes[3]
    vz = data.axes[1]
    yawRate = data.axes[0]
    
    
    cmdVelMsg.linear.x = vx
    cmdVelMsg.linear.y = vy
    cmdVelMsg.linear.z = vz
    cmdVelMsg.angular.z = yawRate


    for i in range(0, len(data.buttons)):
        if buttons == None or data.buttons[i] != buttons[i]:
            if i == 0 and data.buttons[i] == 1: # and flag_land != None:
                land()
                print("joy: land")
            if i == 2 and data.buttons[i] == 1: # and flag_takeoff != None:
                takeoff()
                print("joy: take off")
                        
    buttons = data.buttons 

#-----------------------------------------------------------------------------




# subscribers
# ------------
rospy.Subscriber("/joy", Joy, callBackJoy)



# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    while not rospy.is_shutdown():
        # msg publication
        pubCmdVel.publish(cmdVelMsg)
        cmdPubRate.sleep()
# -----------------------------------------------------------------------------