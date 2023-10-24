#!/usr/bin/env python3

import sys
import copy
import rospy
from time import sleep
from std_msgs.msg import Bool
from std_msgs.msg import String
from gripper.msg import gripper_output as gripper_outputMsg
from gripper.msg import gripper_feedback as gripper_feedbackMsg
autoMode = False
sensorDataString = ""
feedbackDataString = ""

# Function named "autoMode_callback". The function takes one parameter, "msg", which is expected to be an object with a .data field. Within the function, a global variable named "autoMode" is assigned to the contents of the .data field of the passed "msg" object.
def autoMode_callback(msg):
    global autoMode
    autoMode = msg.data

# Function named "feedback_callback". The function takes one parameter, "msg", which is an object which holds sensory data and the function will create a string from this data and put it in global variable "feedbackDataString"
def feedback_callback(msg):
    global feedbackDataString
    feedbackData = msg
    feedbackDataString = ("x_gripperOpened" + "," + str(feedbackData.x_gripperOpened) + "," + "x_gripperClosed" + "," + str(feedbackData.x_gripperClosed))

# Function named "gripper_callback". The function takes one parameter, "msg", which is an object which holds data and the function will create a string from this data and put it in global variable "gripperOutput"
def gripper_callback(msg):
    global gripperOutput
    gripperOutput = msg

# Function named "gripper_callback_nodered". That receives a string out of NodeRed. It will split this string and checks if the forward, backward, left or right button is activated in NodeRed and put it in global variable "gripperOutput".
def gripper_callback_nodered(msg):
    global gripperOutput
    stringData = msg.data
    arrayData = [x.strip() for x in stringData.strip().split(',')]
    if (arrayData[1] == "true"):
    	gripperOutput.x_gripperOpen = True
    else:
    	gripperOutput.x_gripperOpen = False
    if (arrayData[3] == "true"):
    	gripperOutput.x_gripperClose = True
    else:
    	gripperOutput.x_gripperClose = False

if __name__=='__main__':
    # Initialize node
    rospy.init_node("gripper_node", anonymous=True)
    # rospy.publisher(where to publish, how the message looks, the message size)
    pubOutput = rospy.Publisher("topic_gripper_output", gripper_outputMsg, queue_size=10)
    pubFeedback = rospy.Publisher("topic_gripper_feedback_nodered", String, queue_size=10)
    gripperOutput = gripper_outputMsg()
    feedbackData = gripper_feedbackMsg()
    rospy.loginfo("gripper node opgestart")
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        # rospy.Subscriber(where to publish, how the message looks, function)
        rospy.Subscriber("topic_gripper_feedback", gripper_feedbackMsg, feedback_callback)
        pubFeedback.publish(feedbackDataString)
        rospy.Subscriber("topic_autoMode", Bool, autoMode_callback)
        if autoMode == True:
            rospy.Subscriber("topic_gripper_automatic_output", gripper_outputMsg, gripper_callback)
        else:
            rospy.Subscriber("topic_gripper_manual_output", String, gripper_callback_nodered)

        pubOutput.publish(gripperOutput)   
