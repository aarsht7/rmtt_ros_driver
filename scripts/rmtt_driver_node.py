#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 20 2022

@author: Sylvain BERTRAND
"""

#import time
import robomaster
from robomaster import robot

import rospy
import numpy as np
from std_msgs.msg import Int8, Float32, Empty, ColorRGBA
from geometry_msgs.msg import Vector3, Twist
from std_srvs.srv import Trigger, TriggerResponse

#import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


# node init
# =============================================================================
rospy.init_node('rmtt_driver', anonymous=False)



# main variables
# =============================================================================
drone = robot.Drone()

frequency = 100.0
Ts = 1.0/frequency
nodeRate = rospy.Rate(frequency)



# ROS parameters
# =============================================================================
IP_ADDRESS_STR = rospy.get_param('~IP_ADRESS_STR', "192.168.10.2")
V_XY_MAX = rospy.get_param('~V_XY_MAX', 40)   # betwen 0 and 100
V_Z_MAX = rospy.get_param('~V_Z_MAX', 60)
V_YAW_RATE_MAX = rospy.get_param('~V_YAW_RATE_MAX', 50)
ACTIVE_FRONT_CAM = rospy.get_param('~ACTIVE_FRONT_CAM', True)
FRONT_CAM_FREQ = rospy.get_param('~FRONT_CAM_FREQ', 50.0)
ACTIVE_MISSION_PAD = rospy.get_param('~ACTIVE_MISSION_PAD', True)
MISSION_PAD_FREQ = rospy.get_param('~MISSION_PAD_FREQ', 20.0)



# init global variables
# =============================================================================
global drone_state, battery_state
drone_state = "LANDED"  # LANDED, FLYING
battery_state = "NA"  # NA, GOOD, POOR, CRITICAL

bridge = CvBridge()



# ROS publishers
# =============================================================================

pubBottomTof = rospy.Publisher('btm_range', Float32, queue_size=10)
pubForwardTof = rospy.Publisher('fwd_range', Float32, queue_size=10)
pubRollPitchYawAngle = rospy.Publisher('roll_pitch_yaw_deg', Vector3, queue_size=10)
pubVel = rospy.Publisher('vel', Vector3, queue_size=10)
pubAcc = rospy.Publisher('acc', Vector3, queue_size=10)
pubMissionPadXYZ = rospy.Publisher('mission_pad/xyz', Vector3, queue_size=10)
pubMissionPadID = rospy.Publisher('mission_pad/pad_id', Int8, queue_size=10)
pubMissionPadYawDeg = rospy.Publisher('mission_pad/yaw_deg', Float32, queue_size=10)
pubBatterySoc = rospy.Publisher('battery', Float32, queue_size=10)
pubFrontCam = rospy.Publisher('front_cam/image_raw', Image, queue_size=10)
pubFrontCamInfo = rospy.Publisher('front_cam/camera_info', CameraInfo, queue_size=10) # WORK IN PROGRESS


# call backs for ROS subscribers
# =============================================================================

# -----------------------------------------------------------------------------
def callBackTakeOff(req):
# -----------------------------------------------------------------------------
    global drone_state
    resp = TriggerResponse(False,'')
    if (drone_state=="LANDED"):
        #try:
        rospy.loginfo("     Taking off ...")
        drone.led.set_led_breath(freq=2, r=0, g=255, b=0)    
        drone.flight.takeoff().wait_for_completed()
        drone_state="FLYING"
        drone.led.set_led(r=0, g=255, b=0)    
        rospy.loginfo("                ... Flying")
        resp = TriggerResponse(True,'')
    return resp
        #except rospy.ServiceException as e :
        #    rospy.logerror(e)
        #    resp = TriggerResponse(True,e)
        #    return resp

# -----------------------------------------------------------------------------
def callBackLand(req):
# -----------------------------------------------------------------------------
    global drone_state
    resp = TriggerResponse(False,'')
    if (drone_state=="FLYING"):
        #try:
        rospy.loginfo("     Landing ...")
        drone.led.set_led_breath(freq=2, r=255, g=0, b=0)    
        drone.flight.land().wait_for_completed()
        drone_state="LANDED"
        drone.led.set_led(r=0, g=0, b=0)    
        rospy.loginfo("             ... Landed")
        resp = TriggerResponse(True,'')
    return resp
        #except rospy.ServiceException as e :
        #    rospy.logerror(e)
        #    resp = TriggerResponse(True,e)
        #    return resp
        
def callBackEmergency(req):
# -----------------------------------------------------------------------------
    global drone_state
    resp = TriggerResponse(False,'')
    if (drone_state=="FLYING"):
        #try:
        rospy.loginfo("     EMERGENCY REQUESTED ...")
        drone.led.set_led_breath(freq=0.5, r=255, g=0, b=0)    
        drone.flight.land().wait_for_completed()
        drone_state="LANDED"
        resp = TriggerResponse(True,'')
    return resp
        #except rospy.ServiceException as e :
        #    drone_state="LANDED"
        #    drone.close()
        #    rospy.logerror(e)
        #    resp = TriggerResponse(True,e)
        #    return resp

# -----------------------------------------------------------------------------
def callBackCmdVel(data):
# -----------------------------------------------------------------------------
    # cmdvel linear(x,y,z)  angular(z)  all assumed to be in [-1,1]
    #roll, pitch, accelerate, yaw:  a,b,c,d [-100,100]
    vx = np.rint(100*np.clip(data.linear.x, -1.0, 1.0))
    vy = np.rint(100*np.clip(data.linear.y, -1.0, 1.0))
    vz = np.rint(100*np.clip(data.linear.z, -1.0, 1.0))
    v_yaw_rate = np.rint(100*np.clip(data.angular.z, -1.0, 1.0))
    
    # saturate for safety
    vx = np.clip(vx, -V_XY_MAX, V_XY_MAX)
    vy = np.clip(vy, -V_XY_MAX, V_XY_MAX)
    vz = np.clip(vz, -V_Z_MAX, V_Z_MAX)
    v_yaw_rate = np.clip(v_yaw_rate, -V_YAW_RATE_MAX, V_YAW_RATE_MAX)
    
    if (drone_state=="FLYING"):
        drone.flight.rc(a=-vy, b=vx, c=vz, d=-v_yaw_rate)

# -----------------------------------------------------------------------------
def callBackRGBLed(data):
# -----------------------------------------------------------------------------
    try:
        drone.led.set_led(r=data.r, g=data.g, b=data.b)
    except:
        pass

## -----------------------------------------------------------------------------
#def readFrontCamera(event=None):
## -----------------------------------------------------------------------------
#    img = drone.camera.read_cv2_image()
#    #cv2.imshow("Drone", img)
#    #cv2.waitKey(1)
#    image_message = bridge.cv2_to_imgmsg(img, "rgb8")
#    pubFrontCam.publish(image_message)
#    
#    '''
#    cv_image = bridge.imgmsg_to_cv2(image_message, "rgb8")
#    cv2.imshow("Converted CV to ROS to CV", cv_image)
#    cv2.waitKey(1)
#    '''
#    
#    # WORK IN PROGRESS: to be removed or completed (timestamp)
#    camera_info = CameraInfo()
#    camera_info.header.frame_id = "front_cam"
#    pubFrontCamInfo.publish(camera_info)
#    
    

# -----------------------------------------------------------------------------
#def readMissionPadInfo(event=None):
## -----------------------------------------------------------------------------
#    padMID = drone.get_status("mid")
#    if (padMID>0):
#        padX = drone.get_status("x")
#        padY = drone.get_status("y")
#        padZ = drone.get_status("z")
#        padPRY = drone.get_status("mpry")
#        padYawDeg = padPRY[1]
#    else:
#        padX = None
#        padY = None
#        padZ = None
#        padYawDeg = None
#    #print("     Mission Pad:  ID {0}  x {1}  y {2}  z {3} yaw_deg{4}".format(padMID, padX, padY, padZ, padYawDeg))
#    pubMissionPadID.publish(Int8(int(padMID)))
#    pubMissionPadYawDeg.publish(Float32(padYawDeg))
#    pubMissionPadXYZ.publish(Vector3(padX, padY, padZ))
#    
    
    

# ROS subscribers
# =============================================================================
#rospy.Subscriber("takeoff", Empty, callBackTakeOff)
#rospy.Subscriber("land", Empty, callBackLand)
rospy.Subscriber("cmd_vel", Twist, callBackCmdVel)
rospy.Subscriber("rgb_led", ColorRGBA, callBackRGBLed)



# handlers to SDK
# =============================================================================


# ----- #bottom TOF sensor (in meters) ----------------------------------------
def sub_bottom_tof_info_handler(tof_cm):         
# -----------------------------------------------------------------------------
    
    tof_fwd_cm = drone.sensor.get_ext_tof()
    
    #print("     drone bottom tof (cm): {0}".format(tof_cm))
    if (tof_cm>0):
        pubBottomTof.publish(Float32(tof_cm/100.0))
    else:
        pubBottomTof.publish(Float32(0.0))

    #print("     drone forward tof (cm): {0}".format(tof_fwd_cm))
    if (tof_fwd_cm==None):
        pubForwardTof.publish(Float32(np.nan))
    else:
        if (tof_fwd_cm>0):
            pubForwardTof.publish(Float32(tof_fwd_cm/100.0))
        else:
            pubForwardTof.publish(Float32(0.0))
       

# ----- roll, pitch, yaw angles (in  deg) -------------------------------------
def sub_attitudeRPY_info_handler(attitute_angles):
# -----------------------------------------------------------------------------
    yaw, pitch, roll = attitute_angles
    #print("     drone attitude (deg): roll:{0}, pitch:{1}, yaw:{2} ".format(roll, pitch, yaw))
    pubRollPitchYawAngle.publish(Vector3(roll,pitch,yaw))


# --- accelerations in body frame (m/s2) and velocity in world frame (m/S  TBC)  ----
def sub_imu_info_handler(imu_info):
# -----------------------------------------------------------------------------
    vgx, vgy, vgz, agx, agy, agz = imu_info
    agx = 0.01*agx
    agy = 0.01*agy
    agz = 0.01*agz
    #print("     drone speed (unit ?): vgx {0}, vgy {1}, vgz {2}".format(vgx, vgy, vgz))
    #print("     drone acceleration (m/s2): agx {0}, agy {1}, agz {2}".format(agx, agy, agz))
    pubVel.publish(Vector3(vgx,vgy,vgz))
    pubAcc.publish(Vector3(agx,agy,agz))


# ----- battery level ---------------------------------------------------------
def sub_battery_info_handler(battery_info):
# -----------------------------------------------------------------------------
    global battery_state
    battery_soc = battery_info
    pubBatterySoc.publish(Float32(battery_soc))
    #print("Drone battery: soc {0}".format(battery_soc))
    
    # warnings for different levels of battery state of charge
    if (battery_state=="NA"):  # Not Available
        if (battery_soc<30):
            battery_state = "MEDIUM"
            print("  battery: {0}".format(battery_soc))
    
    if (battery_state=="MEDIUM"):
        if (battery_soc<20):
            battery_state = "POOR"
            print("  [WARNING]  battery: {0}".format(battery_soc))
            
    if (battery_state=="POOR"):
        if (battery_soc<10):
            battery_state = "CRITICAL" 
            print("  [ALERT]  battery: {0}".format(battery_soc))




# main node loop
# =============================================================================

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    
    robomaster.config.LOCAL_IP_STR = IP_ADDRESS_STR
    
    print("\n**** RMTT ROS DRIVER ****")
    drone.initialize()
    print("  connexion to "+IP_ADDRESS_STR+" ..... ok")
    drone_version = drone.get_sdk_version()
    print("  drone sdk version: {0}".format(drone_version))
    print("  Ready to fly!")
    
    
    drone.sub_tof(freq=10, callback=sub_bottom_tof_info_handler)
    drone.flight.sub_attitude(10, sub_attitudeRPY_info_handler)
    drone.flight.sub_imu(10, sub_imu_info_handler)
    drone.battery.sub_battery_info(freq=1, callback=sub_battery_info_handler)
    
    drone.led.set_mled_graph('0000000000000000000000000000000000000000000000000000000000000000')
    
    #if (ACTIVE_FRONT_CAM):
    #    drone.camera.start_video_stream(display=False)
    #    drone.camera.set_fps("high")
    #    drone.camera.set_resolution("high")
    #    drone.camera.set_bitrate(6)  
    #    rospy.Timer(rospy.Duration(1.0/FRONT_CAM_FREQ), readFrontCamera)
    #
    #if (ACTIVE_MISSION_PAD):
    #    drone.flight.mission_pad_on()
    #    rospy.Timer(rospy.Duration(1.0/MISSION_PAD_FREQ), readMissionPadInfo)
    takeoff = rospy.Service('takeoff', Trigger, callBackTakeOff)
    land = rospy.Service('land', Trigger, callBackLand)
    emergency = rospy.Service('emergency', Trigger, callBackEmergency)
    
    rospy.spin()    
    
    
    '''
    while not rospy.is_shutdown():
        
        
#        tof_info = drone.sensor.get_ext_tof()
        
#        print("ext tof: {0}".format(tof_info))
        
        if (ACTIVE_FRONT_CAM):
            img = drone.camera.read_cv2_image()
            cv2.imshow("Drone", img)
            cv2.waitKey(1)
      
        
        nodeRate.sleep()
    '''
    
    
      
    #cv2.destroyAllWindows()  


    drone.unsub_tof()
    drone.flight.unsub_attitude()
    drone.flight.unsub_imu()
    drone.battery.unsub_battery_info()
    

    #if (ACTIVE_FRONT_CAM):
    #    drone.camera.stop_video_stream()
    #if (ACTIVE_MISSION_PAD):
    #    drone.flight.mission_pad_off()

    drone.close()



