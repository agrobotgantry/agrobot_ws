#!/usr/bin/env python3

import sys
import copy
import rospy
import numpy as np
import imutils
import cv2
from time import sleep
from std_msgs.msg import Bool
from position_crop.msg import crop_position as position_cropMsg

pixelHeight = 540
pixelWidth = 960
pixelRange = 50

positionCropAsked = False

cap = cv2.VideoCapture(2)

def positionCropAsked_callback(msg):
    global positionCropAsked
    positionCropAsked = msg.data
    
def thresholding(img):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lowerGreen = np.array([33, 30, 0])
    upperGreen = np.array([87, 255, 255])
    maskGreen = cv2.inRange(imgHsv, lowerGreen, upperGreen)
    return maskGreen

def centor(img, thresh):
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    isFirstCrop = True
    
    for c in cnts:
        # compute the center of the contour
        M = cv2.moments(c)
        area = cv2.contourArea(c)
        if area > 10000:
            print(area)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            # draw the contour and center of the shape on the image
            img = img.copy()
            if isFirstCrop == True:
                cv2.drawContours(img, [c], -1, (0, 0, 0), 2)
            else:
                cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
            cv2.circle(img, (cX, cY), 3, (0, 0, 255), -1)
            cv2.putText(img, "center", (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            if isFirstCrop == True:
                positionCrop.x_cropDetected = True
                #put black point in the middle of the screen
                cv2.circle(img, (720, 270), 3, (255, 255, 255), -1)
                if cX > ((pixelWidth * 0.75) + pixelRange):
                    #rospy.loginfo("Portaal naar Links (-x)")
                    positionCrop.x_XgoPositive = False
                    positionCrop.x_XisRight = False
                    positionCrop.x_XgoNegative = True
                elif cX < ((pixelWidth * 0.75) - pixelRange):
                    #rospy.loginfo("Portaal naar Rechts (+x)")
                    positionCrop.x_XgoNegative = False
                    positionCrop.x_XisRight = False
                    positionCrop.x_XgoPositive = True
                else:
                    #rospy.loginfo("Gewas is op x-as in midden")
                    positionCrop.x_XgoNegative = False
                    positionCrop.x_XgoPositive = False
                    positionCrop.x_XisRight = True
                if cY < (((pixelHeight / 2) - 30) - pixelRange):
                    #rospy.loginfo("Portaal omhoog (+y)")
                    positionCrop.x_YgoNegative = False
                    positionCrop.x_YisRight = False
                    positionCrop.x_YgoPositive = True
                elif cY > (((pixelHeight / 2) - 30) + pixelRange):
                    #rospy.loginfo("Portaal omlaag (-y)")
                    positionCrop.x_YgoPositive = False
                    positionCrop.x_YisRight = False
                    positionCrop.x_YgoNegative = True
                else:
                    #rospy.loginfo("Gewas is op y-as in midden")
                    positionCrop.x_YgoPositive = False
                    positionCrop.x_YgoNegative = False
                    positionCrop.x_YisRight = True
                isFirstCrop = False
    return img

if __name__=='__main__':
    # Initialize node
    rospy.init_node("position_crop_node", anonymous=True)
    # rospy.publisher(where to publish, how the message looks, the message size)
    pub = rospy.Publisher("/topic_position_crop", position_cropMsg, queue_size=10)
    positionCrop = position_cropMsg()
    rospy.loginfo("position crop node opgestart")
    while not rospy.is_shutdown():
        rospy.Subscriber("/position_crop_asked", Bool, positionCropAsked_callback)
        if positionCropAsked == True:
            positionCrop.x_cropDetectionActive = True
            positionCrop.x_cropDetected = False
            #rospy.loginfo("uitlezen img")
            success, img = cap.read()
            img = cv2.resize(img, (960, 540))
            imgflip = cv2.flip(img, 0)
            imgThres = thresholding(imgflip)
            kernel = np.ones((5, 5), np.uint8)
            imgClosing = cv2.morphologyEx(imgThres, cv2.MORPH_CLOSE, kernel)
            imgClosing2 = cv2.morphologyEx(imgClosing, cv2.MORPH_CLOSE, kernel)
            imgCentor = centor(imgflip, imgClosing2)
            imgflip2 = cv2.flip(imgCentor, 1)
            #cv2.imshow('ResultClosing', imgClosing)
            #cv2.imshow('ResultThres', imgThres)
            #cv2.imshow('ResultClosing2', imgClosing2)
            #cv2.imshow('ResultCentor', imgCentor)
            #cv2.imshow('Result', imgflip2)
            cv2.waitKey(1)
        else:
            positionCrop.x_cropDetectionActive = False
            positionCrop.x_cropDetected = False
            positionCrop.x_XgoNegative = False
            positionCrop.x_XgoPositive = False
            positionCrop.x_XisRight = False
            positionCrop.x_YgoPositive = False
            positionCrop.x_YgoNegative = False
            positionCrop.x_YisRight = False

        pub.publish(positionCrop)
        sleep(0.1)
