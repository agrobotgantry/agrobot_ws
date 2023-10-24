#!/usr/bin/env python3

import sys
import copy
import rospy
from time import sleep
import cv2
import os
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras import datasets, layers, models
from std_msgs.msg import Bool
from std_msgs.msg import String
cap = cv2.VideoCapture(3)

recogniseCropAsked = False
recogniseCropOld = ""
recogniseCrop = ""

def recogniseCropAsked_callback(msg):
    global recogniseCropAsked
    recogniseCropAsked = msg.data

# Function that expects a filepath and a modelname.keras file. It returns a neural network in a model.keras file format
def load_model(path, modelname):
    modelpath = path
    mod = modelname
    model = models.load_model(os.path.join(modelpath, mod))
    return model
    
# Function expects a image and a model.keras file. returns a label in string format ['Beetroot', 'Carrot', 'Lettuce', 'Radish']
def predict_image2(image, model):
    img = image/255 # normalize
    img = np.expand_dims(img,axis=0) # Expand with extra dimension
    predict = np.argmax(CNN.predict(img),axis=1) # predict label
    # print(predict) (mag vlgm weg)
    if predict[0] == 0:
        label = 'Beetroot'
    elif predict[0] == 1:
       label = 'Carrot'
    elif predict[0] == 2:
        label = 'Lettuce'
    else:
        label = 'Radish'
    return label
           
    
if __name__=='__main__':
    #rospy.loginfo("recognise crop node opgestart")
    rospy.init_node("recognise_crop_node", anonymous=True)
    pub = rospy.Publisher("/topic_recognise_crop", String, queue_size=10)
    while not rospy.is_shutdown():
        rospy.sleep(0.05)
        rospy.Subscriber("/recognise_crop_asked", Bool, recogniseCropAsked_callback)
        if recogniseCropAsked == True:
            success, img = cap.read()
            img = cv2.resize(img, (250, 250))
            CNN = load_model('agrobot_ws/src/recognise_crop/src','Vegetable_recognition2.keras') # If needed change the filepath here
            label = predict_image2(img, CNN)
            recogniseCrop = label
            #cv2.imshow('Result', img)
            cv2.waitKey(1)
            if recogniseCrop == recogniseCropOld:
                rospy.loginfo(recogniseCrop)
                pub.publish(recogniseCrop)
            else:
                recogniseCropOld = recogniseCrop
        else:
            recogniseCrop = "Noting"
            pub.publish(recogniseCrop) 
        
