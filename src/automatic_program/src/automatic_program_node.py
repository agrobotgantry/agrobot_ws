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
from portal.msg import portal_output as portal_outputMsg
from portal.msg import portal_sensors as portal_sensorsMsg
from portal.msg import portal_feedback as portal_feedbackMsg
from gripper.msg import gripper_output as gripper_outputMsg
from gripper.msg import gripper_feedback as gripper_feedbackMsg
from position_crop.msg import crop_position as position_cropMsg
from geometry_msgs.msg  import PoseStamped as uwb_positieMsg

driveAutomaticOutput = drive_outputMsg()
driveSensorData = drive_sensorsMsg()
driveFeedbackData = drive_feedbackMsg()

portalAutomaticOutput = portal_outputMsg()
portalSensorData = portal_sensorsMsg()
portalFeedbackData = portal_feedbackMsg()
   
gripperAutomaticOutput = gripper_outputMsg()
gripperFeedbackData = gripper_feedbackMsg()
    
positionCropData = position_cropMsg()
   
uwbRightPosition = uwb_positieMsg()
uwbLeftPosition = uwb_positieMsg()

autoMode = False
positionCropAsked = False
recogniseCropAsked = False
recogniseCropData = ""
step = 0
targetBox = 1
targetPosition = 0

startPosition = 1.00
positionBox1 = 3.50
positionBox2 = 5.00

widthBox = 1.00
positionMovement = 0.2

# 
def autoMode_callback(msg):
    global autoMode
    autoMode = msg.data

def drive_sensors_callback(msg):
    global driveSensorsData
    driveSensorData = msg
    
def drive_feedback_callback(msg):
    global driveFeedbackData
    feedbackData = msg

def portal_sensors_callback(msg):
    global portalSensorData
    portalSensorData = msg
    
def portal_feedback_callback(msg):
    global portalFeedbackData
    portalFeedbackData = msg
    
def gripper_feedback_callback(msg):
    global gripperFeedbackData
    gripperFeedbackData = msg
    
def position_crop_callback(msg):
    global positionCropData
    positionCropData = msg

def recognise_crop_callback(msg):
    global recogniseCrop
    recogniseCropData = msg

def uwbLeftPosition_callback(msg):
    global uwbLeftPosition
    uwbLeftPosition = msg

def uwbRightPosition_callback(msg):
    global uwbRightPosition
    uwbRightPosition = msg

   

if __name__=='__main__':
# (INITIALISEREN NODE)
    # zorgt voor meerdere topics subscriben 
    rospy.init_node("automaticProgram_node", anonymous=True)
    # Waar publish je op, hoe ziet de mnessage eruit, grote van message
    pubDriveOutput = rospy.Publisher("topic_drive_automatic_output", drive_outputMsg, queue_size=10)
    pubPortalOutput = rospy.Publisher("topic_portal_automatic_output", portal_outputMsg, queue_size=10)
    pubGripperOutput = rospy.Publisher("topic_gripper_automatic_output", gripper_outputMsg, queue_size=10)
    pubPositionCropAsked = rospy.Publisher("/position_crop_asked", Bool, queue_size=10)
    pubRecogniseCropAsked = rospy.Publisher("/recognise_crop_asked", Bool, queue_size=10)

    startPosition = 3.0
    targetBox = 1
    targetPosition = positionBox1
    rospy.loginfo("node opgestart")
    while not rospy.is_shutdown():
    # Wat die uitvoert
        rospy.sleep(0.1)
        # Voer functie 1 aan
        rospy.Subscriber("topic_autoMode", Bool, autoMode_callback)
        
        #rospy.loginfo(autoMode)
        if autoMode == True:
            rospy.Subscriber("topic_drive_sensors", drive_sensorsMsg, drive_sensors_callback)
            rospy.Subscriber("topic_drive_feedback", drive_feedbackMsg, drive_feedback_callback)
            rospy.Subscriber("topic_portal_sensors", portal_sensorsMsg, portal_sensors_callback)
            rospy.Subscriber("topic_portal_feedback", portal_feedbackMsg, portal_feedback_callback)
            rospy.Subscriber("topic_gripper_feedback", gripper_feedbackMsg, gripper_feedback_callback)
            rospy.Subscriber("topic_position_crop", position_cropMsg, position_crop_callback)
            rospy.Subscriber("topic_recognise_crop", String, recognise_crop_callback)
            rospy.Subscriber("dwm1001/tag/tagLeft/position", uwb_positieMsg, uwbLeftPosition_callback)
            rospy.Subscriber("dwm1001/tag/tagRight/position", uwb_positieMsg, uwbRightPosition_callback)
            #rospy.loginfo(step)
            driveAutomaticOutput.i_targetSpeed = 75

            #step 0: Move Agrobot to start position
            if step == 0:
                driveAutomaticOutput.x_moveBackwards = True
                if (uwbLeftPosition.pose.position.y or uwbRightPosition.pose.position.y) < startPosition:
                    driveAutomaticOutput.x_moveBackwards = False
                    step = (step + 1)
            #step 1: Move portal to home position
            if step == 1:
                portalAutomaticOutput.x_speedFast = True
                portalAutomaticOutput.x_goHomePosition = True
                if portalFeedbackData.x_isHomePosition == True or (portalSensorData.x_ZendSwitchUp and portalSensorData.x_XendSwitchNegative and portalSensorData.x_YendSwitchNegative):
                    portalAutomaticOutput.x_speedFast = False
                    portalAutomaticOutput.x_goHomePosition = False
                    step = (step + 1)
            #step 2: Move Agrobot to the targetposition
            if step == 2:
                driveAutomaticOutput.x_moveForward = True
                if (uwbLeftPosition.pose.position.y or uwbRightPosition.pose.position.y) > targetPosition:
                    driveAutomaticOutput.x_moveForward = False
                    step = (step + 1)
            #step 3: Check if Z-axis is up
            if step == 3:    
                portalAutomaticOutput.x_ZgoUp = True
                if portalSensorData.x_ZendSwitchUp:
                    portalAutomaticOutput.x_ZgoUp = False
                    step = (step + 1)
            #step 4: Check if gripper is open
            if step == 4:       
                gripperAutomaticOutput.x_gripperOpen = True
                if gripperFeedbackData.x_gripperOpened:
                    gripperAutomaticOutput.x_gripperOpen = False
                    step = (step + 1)
            #step 5: Set vision on and check it is on
            if step == 5:
                positionCropAsked = True
                portalAutomaticOutput.x_positionLightOn = True
                portalAutomaticOutput.x_recogniseLightOn = True  
                if positionCropData.x_cropDetectionActive:
                    step = (step + 1)
            #step 6: Positioneer to crop
            if step == 6:
                if positionCropData.x_cropDetected:
                    portalAutomaticOutput.x_speedFast = False
                    portalAutomaticOutput.x_XgoPositive = positionCropData.x_XgoPositive 
                    portalAutomaticOutput.x_XgoNegative = positionCropData.x_XgoNegative
                    portalAutomaticOutput.x_YgoPositive = positionCropData.x_YgoPositive 
                    portalAutomaticOutput.x_YgoNegative = positionCropData.x_YgoNegative
                    rospy.loginfo(positionCropData)
                    if portalSensorData.x_YendSwitchPositive and portalAutomaticOutput.x_YgoPositive:
                        portalAutomaticOutput.x_XgoPositive = True
                else:
                    portalAutomaticOutput.x_speedFast = True
                    portalAutomaticOutput.x_XgoPositive = True   
                if positionCropData.x_XisRight and positionCropData.x_YisRight:
                    portalAutomaticOutput.x_XgoPositive = False
                    portalAutomaticOutput.x_XgoNegative = False
                    portalAutomaticOutput.x_YgoPositive = False
                    portalAutomaticOutput.x_YgoNegative = False
                    step = (step + 1)
                if portalSensorData.x_XendSwitchPositive:
                    targetPosition = targetPosition + positionMovement
                    portalAutomaticOutput.x_XgoPositive = False
                    portalAutomaticOutput.x_XgoNegative = False
                    portalAutomaticOutput.x_YgoPositive = False
                    portalAutomaticOutput.x_YgoNegative = False
                    if (targetBox == 1) and (targetPosition > (positionBox1 + widthBox)):
                        targetBox = 2
                        targetPosition = positionBox2
                        step = 1
                    elif (targetBox == 2) and (targetPosition > (positionBox2 + widthBox)):
                        targetBox = 3
                    else:
                        step = 1
            #step 7: Check if the portal is position right and portal is not moving any more
            if step == 7:
                if not (portalFeedbackData.x_XgoingPositive or portalFeedbackData.x_XgoingNegative or portalFeedbackData.x_YgoingPositive or portalFeedbackData.x_YgoingNegative):
                    if positionCropData.x_XisRight and positionCropData.x_YisRight:
                        step = (step + 1)  
                    else:
                        step = (step - 1)
                else:
                    portalAutomaticOutput.x_XgoPositive = False
                    portalAutomaticOutput.x_XgoNegative = False
                    portalAutomaticOutput.x_YgoPositive = False
                    portalAutomaticOutput.x_YgoNegative = False
            #step 8: Disable vision (to make Jetson faster)
            if step == 8:
                positionCropAsked = False
                portalAutomaticOutput.x_positionLightOn = False
                portalAutomaticOutput.x_recogniseLightOn = False
                if positionCropData.x_cropDetectionActive == False:
                    step = (step + 1)
            #step 9: Check if Z-axis is still up
            if step == 9:
                portalAutomaticOutput.x_ZgoUp = True
                if portalSensorData.x_ZendSwitchUp:
                    portalAutomaticOutput.x_ZgoUp = False
                    step = (step + 1)
            #step 10: set Z-axis down             
            if step == 10:
                portalAutomaticOutput.x_ZgoDown = True
                if portalSensorData.x_ZendSwitchDown:
                    portalAutomaticOutput.x_ZgoDown = False
                    step = (step + 1)
            #step 11: Move X-axis    
            if step == 11:    
                portalAutomaticOutput.x_speedFast = False
                portalAutomaticOutput.x_XgoPositive = True
                if portalFeedbackData.x_XgoingPositive:
                    step = (step + 1)
            #step 12: sleep 1 second to  move the X-axis
            if step == 12:
                rospy.sleep(1)
                step = (step + 1)
            #step 13: Stop moving the X-axis
            if step == 13:
                portalAutomaticOutput.x_XgoPositive = False
                if portalFeedbackData.x_XgoingPositive == False:
                    step = (step + 1)
            #step 14: Close the gripper        
            if step == 14:
                gripperAutomaticOutput.x_gripperClose = True
                if gripperFeedbackData.x_gripperClosed:
                    gripperAutomaticOutput.x_gripperClose = False
                    step = (step + 1)
            #step 15: Z-axis up
            if step == 15:    
                portalAutomaticOutput.x_ZgoUp = True
                if portalSensorData.x_ZendSwitchUp:
                    portalAutomaticOutput.x_ZgoUp = False
                    step = (step + 1)
            #step 16: Homing the portal
            if step == 16:
                portalAutomaticOutput.x_speedFast = True
                portalAutomaticOutput.x_goHomePosition = True
                if portalFeedbackData.x_isHomePosition == True:
                    portalAutomaticOutput.x_speedFast = False
                    portalAutomaticOutput.x_goHomePosition = False
                    step = (step + 1)
            #step 17: Move backwards
            if step == 17:
                driveAutomaticOutput.x_moveBackwards = True
                if (uwbLeftPosition.pose.position.y or uwbRightPosition.pose.position.y) < 2.50:
                    driveAutomaticOutput.x_moveBackwards = False
                    step = (step + 1)

            #step 18: Move the X-axis to the AI photo location        
            if step == 18:
                portalAutomaticOutput.x_speedFast = True
                portalAutomaticOutput.x_XgoPositive = True
                if portalFeedbackData.x_XgoingPositive:
                    step = (step + 1)
            #step 19: Sleep for 12 seconds to move the X-axis
            if step == 19:
                rospy.sleep(17)
                step = (step + 1)
            #step 20: Stop moving the X-axis
            if step == 20:
                portalAutomaticOutput.x_speedFast = False
                portalAutomaticOutput.x_XgoPositive = False
                if portalFeedbackData.x_XgoingPositive == False:
                    step = (step + 1)    
            #step 21: Move the Y-axis to the AI photo location
            if step == 21:
                portalAutomaticOutput.x_speedFast = True
                portalAutomaticOutput.x_YgoPositive = True
                if portalFeedbackData.x_YgoingPositive:
                    step = (step + 1)
            #step 22: Sleep for 2 seconds to move the Y-axis.
            if step == 22:
                rospy.sleep(2)
                step = (step + 1)
            #step 23: Stop moving the Y-axis
            if step == 23:  
                portalAutomaticOutput.x_YgoPositive = False
                if portalFeedbackData.x_YgoingPositive == False:
                    step = (step + 1)
            #step 24: set the recognise light on
            if step == 24:
                portalAutomaticOutput.x_recogniseLightOn = True
                if portalFeedbackData.x_recogniseLightIsOn:
                    step = (step + 1)
            #step 25: let AI take photo.
            if step == 25:
                recogniseCropAsked = True
                #Als recognise crop node is opgestart verwijder onderste. 
                recogniseCropData = "Beetroot"
                if recogniseCropData == 'Beetroot':
                    portalAutomaticOutput.x_recogniseLightOn = False
                    step = (step + 1)
                elif recogniseCropData == 'Carrot':
                    portalAutomaticOutput.x_recogniseLightOn = False
                    step = (step + 2)
                elif recogniseCropData == 'Lettuce':
                    portalAutomaticOutput.x_recogniseLightOn = False
                    step = (step + 3)
                elif recogniseCropData == 'Radish':
                    portalAutomaticOutput.x_recogniseLightOn = False
                    step = (step + 4)
            #step 26: Let the gripper open to let go of the beetroot
            if step == 26:
                step = 31 
            #step 27: Move the Y-axis to leto go for position carrot
            if step == 27:        
                portalAutomaticOutput.x_speedFast = True
                portalAutomaticOutput.x_XgoPositive = True
                if portalFeedbackData.x_XgoingPositive:
                    step = (step + 1)
            #step 28: Sleep for 4 seconds to move the X-axis for carrot position
            if step == 28:
                rospy.sleep(4)
                step = (step + 1)
            #step 29: Stop moving the X-axis for carrot position
            if step == 29:
                portalAutomaticOutput.x_speedFast = False
                portalAutomaticOutput.x_XgoPositive = False
                if portalFeedbackData.x_XgoingPositive == False:
                    step = 31
            if step == 30:
                step = 31
            #step 31: open the gripper
            if step == 31:
                gripperAutomaticOutput.x_gripperOpen = True
                if gripperFeedbackData.x_gripperOpened:
                    gripperAutomaticOutput.x_gripperOpen = False
                    step = (1)
                    
            pubDriveOutput.publish(driveAutomaticOutput)
            pubPortalOutput.publish(portalAutomaticOutput)
            pubGripperOutput.publish(gripperAutomaticOutput)
            pubPositionCropAsked.publish(positionCropAsked)
            pubRecogniseCropAsked.publish(recogniseCropAsked)
        
        else:
            #Making drive set false
            driveAutomaticOutput.x_moveForward = False
            driveAutomaticOutput.x_moveBackwards = False
            #Making portal set false
            portalAutomaticOutput.x_XgoPositive = False
            portalAutomaticOutput.x_XgoNegative = False
            portalAutomaticOutput.x_YgoPositive = False 
            portalAutomaticOutput.x_YgoNegative = False
            portalAutomaticOutput.x_ZgoUp = False 
            portalAutomaticOutput.x_ZgoDown = False
            portalAutomaticOutput.x_goHomePosition = False
            #Making gripper set false
 
            pubDriveOutput.publish(driveAutomaticOutput)
            pubPortalOutput.publish(portalAutomaticOutput)
            pubGripperOutput.publish(gripperAutomaticOutput)
        

