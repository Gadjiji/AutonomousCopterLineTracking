#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
A3T's code for the MAAXX 2018 competition in Bristol UK
Student Drone Team from the University of Twente Netherlands


__author__ = "Gabriel Damian"
__copyright__ = "Copyright 2018, A3T"
__credits__ = ["Rob Knight", "Peter Maxwell", "Gavin Huttley",
                    "Matthew Wakefield"]
__license__ = "Open Source"
__version__ = "1.0.1"
__maintainer__ = "Gabriel Damian"
__email__ = "g.d.damian@student.utwente.nl"
__status__ = "Prototype"

The setup consist of a companion computer and a downward 
facing camera (in this case a raspberry pi and pi camera),
a copter and a pixhawk. The connection between the 
companion computer and the flight controler is done via 
telemetry.

The code analyse each frame taken by the camera, finds 
a line on the floor, and give the corresponding correcting
commands to the copter. The commands are degrees per axis
and sent by msg via MAVlink.

CAUTION: this script use the GUIDED_NOGPS mode, which can
give the copter unpredictable behaviors. Be careful and 
keep your distance. As soon as the copter does not behave
as expected, land immediately or cut motors off. 
Thank me later.
"""


#################################################################
#SETUP
#################################################################

#Import modules for vision
import picamera
import picamera.array
import time
import cv2
import numpy as np
from pymavlink import mavutil
import math
import keyboard
import sys

#Import modules for control
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time

# PARAMETERS FOR CAMERA CALIBRATION
minValThreshold = 50      # Threshold for Canny edge detection
accumulatorThreshold = 10 # For tuning line dection
minLineLenght = 80        # In Pixel
maxGap = 0                # Maximum gap allowed between lines 
x11 = 1 # For y max of trend line (lower part of the frame)
x22 = 1 # For y min of trend line (upper part of the frame)
YawPrevious = 0
RollPrevious = 0

#Initialize the camera and grab a reference to the raw camera ca
camera = picamera.PiCamera()
camera.resolution = (640, 480) # Point (0,0) is at the frame's upper left
camera.framerate = 24 

#Take reference frame
rawCapture = picamera.array.PiRGBArray(camera, size=(640, 480))

#Allow camera warmup
time.sleep(0.1)

# Connect to the Vehicle
print 'Connecting to vehicle on: ttyS0'
vehicle = connect('/dev/ttyS0', wait_ready = False, baud = 57600)

# Control Constants and variables
DEFAULT_TAKEOFF_THRUST = 0.7
SMOOTH_TAKEOFF_THRUST = 0.6
TakeOffAltitude = 1.5 #Altitude to reach during takeoff
YawSensibility = 0.05  #Sensibility of the yaw rate
RollSensibility = 0.03 #Sensibility of the roll in degrees
Pitch = 5 #Pitch in degrees when going forward


###################################################################
# Functions
###################################################################

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
	
    print("WARMING: Basic pre-arm checks bypassed")

    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %s" % current_altitude)
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)


def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """
    
    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """
    
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
                                                             0,
                                                             0,
                                                                # Target system
                                                             0,
                                                                 # Target component
                                                             0b00000000,
                                                                 # Type mask: bit 1 is LSB
                                                             to_quaternion(roll_angle, pitch_angle),
                                                                 # Quaternion
                                                             0,
                                                                 # Body roll rate in radian
                                                             0,
                                                                 # Body pitch rate in radian
                                                             math.radians(yaw_rate),
                                                                 # Body yaw rate in radian
                                                             thrust)
                                                                 # Thrust
    vehicle.send_mavlink(msg)
    print("msg sent to copter: Roll %s, Pitch %s, Yaw %s" %(roll_angle, pitch_angle, yaw_rate))

    if duration != 0:
        # Divide the duration into the frational and integer parts
        modf = math.modf(duration)

        # Sleep for the fractional part
        time.sleep(modf[0])

        # Send command to vehicle on 1 Hz cycle
        for x in range(0,int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))
    
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    
    return [w, x, y, z]


###################################################################
# Main
###################################################################

# Take off to target altitude in GUIDED_NOGPS mode.
arm_and_takeoff_nogps(TakeOffAltitude)


#Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        #Grab ref immage
        img = frame.array

        #Convert to Grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #Blur image to reduce noise
        blurred = cv2.GaussianBlur(gray, (9, 9), 0)

        #Perform canny edge-detection
        edged = cv2.Canny(blurred, minValThreshold, minValThreshold*3)

        #Perform hough lines probalistic transform
        lines = cv2.HoughLinesP(edged,1,np.pi/180,accumulatorThreshold,minLineLenght,maxGap)

        #Draw lines on input image
        if(lines is not None):
            for x1,y1,x2,y2 in lines[0]:
                cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)

                if (y1 > 450): x11 = x1
                if (y1 < 50): x22 = x1

        #Draw trend line on input image
        if(lines is not None):
                cv2.line(img,(x11,480),(x22,0),(0,0,225),2)

		Yaw =  -int(YawSensibility*(x11-x22)) #positive is CW negative is CCW
		Roll = -int(RollSensibility*((320-x22)+(320-x11))/2) #left is negative

		#Send flight directive
		set_attitude(roll_angle = Roll, pitch_angle = Pitch, yaw_rate = Yaw, thrust = 0.5, duration = 0)
		print(x11, x22, Yaw, Roll)

	else:
		set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0)
                print("No Line Detected")

        #Show the frame
#        cv2.imshow("Frame", img)
        key = cv2.waitKey(1) & 0xFF

        #Clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        #If the `l` key was pressed, break from the loop
        if keyboard.is_pressed('l'):
		print("Landing")
		break

cv2.waitkey(0)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

# Stopping
print("Disarming")
vehicle.armed = False

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()


print("Completed")


##REFERENCE#
#set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0)
#
# thrust = 0.5 means keep altitude
# duration = 0 means no delay
## 

sys.exit()


