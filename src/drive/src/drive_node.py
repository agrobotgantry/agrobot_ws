#!/usr/bin/env python3

import sys
import copy
import rospy
from time import sleep
from std_msgs.msg import Bool
from std_msgs.msg import String
from drive.msg import drive_output as drive_outputMsg
from drive.msg import drive_sensors as drive_sensorsMsg
from drive.msg import drive_feedback as drive_feedbackMsg
autoMode = False
sensorDataString = ""
feedbackDataString = ""

# Function named "autoMode_callback". The function takes one parameter, "msg", which is expected to be an object with a .data field. Within the function, a global variable named "autoMode" is assigned to the contents of the .data field of the passed "msg" object.
def autoMode_callback(msg):
    global autoMode
    autoMode = msg.data

# Function named "sensors_callback". The function takes one parameter, "msg", which is an object which holds sensory data and the function will create a string from this data and put it in global variable "sensorDataString"
def sensors_callback(msg):
    global sensorDataString
    sensorData = msg
    sensorDataString = ("i_distanceSensorFrontLeft" + "," + str(sensorData.i_distanceSensorFrontLeft) + "," + "i_distanceSensorFrontRight" + "," + str(sensorData.i_distanceSensorFrontRight) + "," + "i_distanceSensorBackLeft" + "," + str(sensorData.i_distanceSensorBackLeft) + "," + "i_distanceSensorBackRight" + "," + str(sensorData.i_distanceSensorBackRight))
   
# Function named "feedback_callback". The function takes one parameter, "msg", which is an object which holds data and the function will create a string from this data and put it in global variable "feedbackDataString"
def feedback_callback(msg):
    global feedbackDataString
    feedbackData = msg
    feedbackDataString = ("x_movingForward" + "," + str(feedbackData.x_movingForward) + "," + "x_movingBackwards" + "," + str(feedbackData.x_movingBackwards) + "," + "x_movingLeft" + "," + str(feedbackData.x_movingLeft) + "," + "x_movingRight" + "," + str(feedbackData.x_movingRight) + "," + "i_currentSpeed" + "," + str(feedbackData.i_currentSpeed))

# Function named "drive_callback". The function takes one parameter, "msg", which is an object which holds sensory data and the function will create a string from this data and put it in global variable "driveOutput"
def drive_callback(msg):
    global driveOutput
    driveOutput = msg

# Function named "drive_callback_nodered". That receives a string out of NodeRed. It will split this string and checks if the forward, backward, left or right button is activated in NodeRed and put it in global variable "driveOutput".
def drive_callback_nodered(msg):
    global driveOutput
    stringData = msg.data
    arrayData = [x.strip() for x in stringData.strip().split(',')]
    if (arrayData[1] == "true"):
    	driveOutput.x_moveForward = True
    else:
    	driveOutput.x_moveForward = False
    if (arrayData[3] == "true"):
    	driveOutput.x_moveBackwards = True
    else:
    	driveOutput.x_moveBackwards = False
    if arrayData[5] == "true":
    	driveOutput.x_moveLeft = True
    else:
    	driveOutput.x_moveLeft = False
    if arrayData[7] == "true":
    	driveOutput.x_moveRight = True
    else:
    	driveOutput.x_moveRight = False
    
    driveOutput.i_targetSpeed = int(arrayData[9])

if __name__=='__main__':
    # Initialize node
    rospy.init_node("drive_node", anonymous=True)
    # rospy.publisher(where to publish, how the message looks, the message size)
    pubOutput = rospy.Publisher("topic_drive_output", drive_outputMsg, queue_size=10)
    pubSensors = rospy.Publisher("topic_drive_sensors_nodered", String, queue_size=10)
    pubFeedback = rospy.Publisher("topic_drive_feedback_nodered", String, queue_size=10)
    driveOutput = drive_outputMsg()
    sensorData = drive_sensorsMsg()
    feedbackData = drive_feedbackMsg()
    rospy.loginfo("drive node opgestart")
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        # rospy.Subscriber(where to publish, how the message looks, function)
        rospy.Subscriber("topic_drive_sensors", drive_sensorsMsg, sensors_callback)
        pubSensors.publish(sensorDataString)
        rospy.Subscriber("topic_drive_feedback", drive_feedbackMsg, feedback_callback)
        pubFeedback.publish(feedbackDataString)
        rospy.Subscriber("topic_autoMode", Bool, autoMode_callback)
        if autoMode == True:
            rospy.Subscriber("topic_drive_automatic_output", drive_outputMsg, drive_callback)

        else:
            rospy.Subscriber("topic_drive_manual_output", String, drive_callback_nodered)

        pubOutput.publish(driveOutput)   
