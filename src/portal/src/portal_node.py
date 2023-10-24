#!/usr/bin/env python3

import sys
import copy
import rospy
from time import sleep
from std_msgs.msg import Bool
from std_msgs.msg import String
from portal.msg import portal_output as portal_outputMsg
from portal.msg import portal_sensors as portal_sensorsMsg
from portal.msg import portal_feedback as portal_feedbackMsg
autoModus = False
sensorDataString = ""
feedbackDataString = ""

# Function named "autoModus_callback". The function takes one parameter, "msg", which is expected to be an object with a .data field. Within the function, a global variable named "autoModus" is assigned to the contents of the .data field of the passed "msg" object.
def autoModus_callback(msg):
    global autoModus
    autoModus = msg.data

# Function named "portal_callback". The function takes one parameter, "msg", which is an object which holds sensory data and the function will create a string from this data and put it in global variable "portalOutput"
def portal_callback(msg):
    global portalOutput
    portalOutput = msg

# Function named "sensors_callback". The function takes one parameter, "msg", which is an object which holds sensory data and the function will create a string from this data and put it in global variable "sensorDataString"
def sensors_callback(msg):
    global sensorData
    global sensorDataString
    sensorData = msg
    sensorDataString = "x_XendSwitchPositive" + "," + str(sensorData.x_XendSwitchPositive) + "," + "x_XendSwitchNegative" + "," + str(sensorData.x_XendSwitchNegative) + "," + "x_YendSwitchPositive" + "," + str(sensorData.x_YendSwitchPositive) + "," + "x_YendSwitchNegative" + "," + str(sensorData.x_YendSwitchNegative) + "," + "x_ZendSwitchUp" + "," + str(sensorData.x_ZendSwitchUp) + "x_ZendSwitchDown" + "," + str(sensorData.x_ZendSwitchDown)

# Function named "feedback_callback". The function takes one parameter, "msg", which is an object which holds data and the function will create a string from this data and put it in global variable "feedbackDataString"    
def feedback_callback(msg):
    global feedbackData
    global feedbackDataString
    feedbackData = msg
    feedbackDataString = "x_XgoingPositive" + "," + str(feedbackData.x_XgoingPositive) + "," + "x_XgoingNegative" + "," + str(feedbackData.x_XgoingNegative) + "," + "x_YgoingPositive" + "," + str(feedbackData.x_YgoingPositive) + "," + "x_YgoingNegative" + "," + str(feedbackData.x_YgoingNegative) + "," + "x_ZgoingUp" + "," + str(feedbackData.x_ZgoingUp) + ","  + "x_ZgoingDown" + "," + str(feedbackData.x_ZgoingDown) + ","  + "x_isHomePosition" + "," + str(feedbackData.x_isHomePosition) + "," + "x_speedFastActive" + "," + str(feedbackData.x_speedFastActive)


# Function named "portal_callback_nodered". That receives a string out of NodeRed. It will split this string and checks if the forward, backward, left or right button is activated in NodeRed and put it in global variable "portalOutput".
def portal_callback_nodered(msg):
    global portalOutput
    stringData = msg.data
    arrayData = [x.strip() for x in stringData.strip().split(',')]
    if (arrayData[1] == "true"):
    	portalOutput.x_XgoPositive = True
    else:
    	portalOutput.x_XgoPositive = False
    if (arrayData[3] == "true"):
    	portalOutput.x_XgoNegative = True
    else:
    	portalOutput.x_XgoNegative = False
    if arrayData[5] == "true":
    	portalOutput.x_YgoPositive = True
    else:
    	portalOutput.x_YgoPositive = False
    if arrayData[7] == "true":
    	portalOutput.x_YgoNegative = True
    else:
    	portalOutput.x_YgoNegative = False
    if arrayData[9] == "true":
    	portalOutput.x_ZgoUp = True
    else:
    	portalOutput.x_ZgoUp = False
    if arrayData[11] == "true":
    	portalOutput.x_ZgoDown = True
    else:
    	portalOutput.x_ZgoDown = False
    if arrayData[13] == "true":
    	portalOutput.x_goHomePosition = True
    else:
    	portalOutput.x_goHomePosition = False
    if arrayData[15] == "true":
    	portalOutput.x_speedFast = True
    else:
    	portalOutput.x_speedFast = False


if __name__=='__main__': 
    # Initialize node
    rospy.init_node("portal_node", anonymous=True)
    # Waar publish je op, hoe ziet de mnessage eruit, grote van message rospy.publisher(where to publish, how the message looks, the message size)    
    pub = rospy.Publisher("topic_portal_output", portal_outputMsg, queue_size=10)
    pubSensors = rospy.Publisher("topic_portal_sensors_nodered", String, queue_size=10)
    pubFeedback = rospy.Publisher("topic_portal_feedback_nodered", String, queue_size=10)
    portalOutput = portal_outputMsg()
    sensorData = portal_sensorsMsg()
    feedbackData = portal_feedbackMsg()
    rospy.loginfo("portal node opgestart")
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        # rospy.Subscriber(where to publish, how the message looks, function)
        rospy.Subscriber("topic_portal_sensors", portal_sensorsMsg, sensors_callback)
        pubSensors.publish(sensorDataString)
        rospy.Subscriber("topic_portal_feedback", portal_feedbackMsg, feedback_callback)
        pubFeedback.publish(feedbackDataString)
        rospy.Subscriber("topic_autoMode", Bool, autoModus_callback)
        if autoModus == True:
            rospy.Subscriber("topic_portal_automatic_output", portal_outputMsg, portal_callback)
        else:
            portalOutput.x_positionLightOn = True
            portalOutput.x_recogniseLightOn = True
            rospy.Subscriber("topic_portal_manual_output", String, portal_callback_nodered)

        pub.publish(portalOutput)  
