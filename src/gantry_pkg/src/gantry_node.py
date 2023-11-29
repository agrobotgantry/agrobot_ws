#! /usr/bin/env python3

# =====================
# Author: Cas Damen & Jerome Kemper
# Created on: 29-11-2023
# Description: Class to control the gantry of the agrobot
# =====================

import rospy
from std_msgs.msg import Empty
from geometry.msg import Point

class Gantry(object):

    # Constructor of the Gantry class
    def __init__(self):
        # Initialise subscribers
        rospy.Subscriber('/start_gantry', Empty, self.start_gantry_callback)

        # Initialise publishers
        self.arduino_initialise_publisher = rospy.Publisher('/agrobot_gantry/initialise', Empty, queue_size=1)
        self.arduino_coordinates_publisher = rospy.Publisher('/agrobot_gantry/coordinates', Point, queue_size=1)

    def start_gantry_callback(self, message):
        #
        # Voeg hier de code die uitgevoerd moet worden wanneer de gantry wordt gestart
        #
        pass

    # Publish command to initialise the gantry
    def publish_initialise(self):
        message = Empty()
        self.arduino_initialise_publisher.publish(message)


    # Publishe the position the Gantry should move to
    def publish_coordinates(self, position_x, position_y, position_z):
        message = Point()
        message.x = position_x
        message.y = position_y
        message.z = position_z
        self.arduino_coordinates_publisher.publish(message)

# Initialise node and call the program
if __name__ == '__main__':
    rospy.init_node('gantry_node')
    Gantry()
    rospy.spin()