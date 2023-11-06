#! /usr/bin/env python3

# =====================
# Author: Cas Damen
# Created on: 31-10-2023
# Description: Class to control the drive and steering of the Agrobot Gantry
# =====================

import rospy
from std_msgs.msg import Float32, Int8

class Drive(object):

    # Constructor of the Drive class
    def __init__(self):
        # Constant which defines the minimal distance an object should be away from the robot (in cm)
        self.MINIMAL_DISTANCE = 30.0

        # Properties which contains the readings of the ultrasonic sensors
        self.ultrasonic_sensor_1 = 0.0
        self.ultrasonic_sensor_2 = 0.0
        self.ultrasonic_sensor_3 = 0.0
        self.ultrasonic_sensor_4 = 0.0

        # Initialise subscribers
        rospy.Subscriber('/agrobot_drive/ultrasonic_1', Float32, self.ultrasonic_sensor_1_callback)
        rospy.Subscriber('/agrobot_drive/ultrasonic_2', Float32, self.ultrasonic_sensor_2_callback)
        rospy.Subscriber('/agrobot_drive/ultrasonic_3', Float32, self.ultrasonic_sensor_3_callback)
        rospy.Subscriber('/agrobot_drive/ultrasonic_4', Float32, self.ultrasonic_sensor_4_callback)

        # Initialise publishers
        self.arduino_command_publisher = rospy.Publisher('/agrobot_drive/arduino_command', Int8, queue_size=1)

    # Method which contains the logic for driving the Agrobot Gantry
    def drive_agrobot(self):
        #
        #
        #
        #
        #

    # Subscriber callback for the distance of ultrasonic sensor 1
    def ultrasonic_sensor_1_callback(self, message):
        self.ultrasonic_sensor_1 = message.data

    # Subscriber callback for the distance of ultrasonic sensor 2
    def ultrasonic_sensor_2_callback(self, message):
        self.ultrasonic_sensor_2 = message.data

    # Subscriber callback for the distance of ultrasonic sensor 3
    def ultrasonic_sensor_3_callback(self, message):
        self.ultrasonic_sensor_3 = message.data

    # Subscriber callback for the distance of ultrasonic sensor 4
    def ultrasonic_sensor_4_callback(self, message):
        self.ultrasonic_sensor_4 = message.data

    # Publish the arduino command
    def publish_arduino_command(self, data):
        self.arduino_command_publisher.Publish(data)


# Initialise node and call the program
if __name__ == '__main__':
    rospy.init_node('drive_node')
    Drive()
    rospy.spin()