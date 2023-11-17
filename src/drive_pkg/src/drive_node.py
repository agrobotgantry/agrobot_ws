#! /usr/bin/env python3

# =====================
# Author: Cas Damen
# Created on: 31-10-2023
# Description: Class to control the drive and steering of the Agrobot Gantry
# =====================

import rospy
from std_msgs.msg import Bool, Int8, String
from geometry.msg import PoseStamped, Point

class Drive(object):

    # Constructor of the Drive class
    def __init__(self):
        # Properties for the locations in the playing field (distances in meters)
        # Those properties are constants and should not be changed during the program
        # A visible presentation of all the positions can be found in drive_pkg/docs/Agrobot Gantry waypoints.png
        self.POSITION_P1 = Point()
        self.POSITION_P1.x = 2.5
        self.POSITION_P1.y = 1.25
        self.POSITION_P2 = Point()
        self.POSITION_P2.x = 2.5
        self.POSITION_P2.y = 5.75
        self.POSITION_P3 = Point()
        self.POSITION_P3.x = 1.0
        self.POSITION_P3.y = 5.75
        self.POSITION_P4 = Point()
        self.POSITION_P4.x = 1.0
        self.POSITION_P4.y = 1.25

        self.POSITION_B11 = Point()
        self.POSITION_B11.x = 2.5
        self.POSITION_B11.y = 2.5
        self.POSITION_B12 = Point()
        self.POSITION_B12.x = 2.5
        self.POSITION_B12.y = 3.5
        self.POSITION_B13 = Point()
        self.POSITION_B13.x = 2.5
        self.POSITION_B13.y = 4.5

        self.POSITION_B21 = Point()
        self.POSITION_B21.x = 1.0
        self.POSITION_B21.y = 2.5
        self.POSITION_B22 = Point()
        self.POSITION_B22.x = 1.0
        self.POSITION_B22.y = 3.5
        self.POSITION_B23 = Point()
        self.POSITION_B23.x = 1.0
        self.POSITION_B23.y = 4.5

        # Property for object detection
        self.object_detected = False

        # Properties for positions of the UWB modules
        self.position_uwb_left = Point()
        self.position_uwb_right = Point()

        # Properties for the center position of the Agrobot Gantry
        self.position_agrobot = Point()
        self.position_agrobot.x = (self.position_uwb_left.x + self.position_uwb_right.x) / 2
        self.position_agrobot.y = (self.position_uwb_left.y + self.position_uwb_right.y) / 2

        # Initialise subscribers
        rospy.Subscriber('/agrobot_object_detection/object_detected', Bool, self.object_detected_callback)
        rospy.Subscriber('/dwm1001/tag/tagLeft/position', PoseStamped, self.uwb_left_callback)
        rospy.Subscriber('/dwm1001/tag/tagRight/position', PoseStamped, self.uwb_right_callback)
        rospy.Subscriber('/drive_to_position', String, self.drive_to_position_callback)

        # Initialise publishers
        self.arduino_drive_command_publisher = rospy.Publisher('/agrobot_drive/arduino_command', Int8, queue_size=1)


    # Drive the Agrobot Gantry forward untill the target position is reached
    def drive_forward_to_target(self, target_position):
        target_reached = False
        orientation = self.get_orientation()

        # Publish in the Arduino command topic to start driving forward
        self.publish_arduino_command(1)

        # Drive untill the target position is reached
        while(not target_reached and not self.object_detected and not rospy.is_shutdown()):
            # Update the position of the Agrobot Gantry
            self.position_agrobot.x = (self.position_uwb_left.x + self.position_uwb_right.x) / 2
            self.position_agrobot.y = (self.position_uwb_left.y + self.position_uwb_right.y) / 2

            if(orientation == 'north'):
                target_reached = self.position_agrobot.y >= target_position.y
            elif(orientation == 'south'):
                target_reached = self.position_agrobot.y <= target_position.y
            elif(orientation == 'west'):
                target_reached = self.position_agrobot.x <= target_position.x
            elif(orientation == 'east'):
                target_reached = self.position_agrobot.x >= target_position.x

        # Publish in the Arduino command topic to stop the Agrobot Gantry
        self.publish_arduino_command(0)

    # Drive the Agrobot Gantry to the next row of vegetables
    def drive_to_next_row(self):
        orientation = self.get_orientation()
        target_position = Point()
        target_position.x = self.position_agrobot.x

        if(orientation == 'north'):
            target_position.y = self.position_agrobot.y + 0.3
        elif(orientation == 'south'):
            target_position.y = self.position_agrobot.y - 0.3

        self.drive_forward_to_target(target_position)

    # Turn the Agrobot Gantry 90 degrees to the left
    def turn_left(self):
        #
        #
        #
        pass

    # Turn the Agrobot Gantry 90 degrees to the right
    def turn_right(self):
        #
        #
        #
        pass

    # Get the orientation the Agrobot Gantry currently has
    # The orientation indicates where the front of the Agrobot Gantry is pointing to
    def get_orientation(self):
        orientation = 'unknown'

        if(abs(self.position_uwb_left.x - self.position_uwb_right.x) > 0.5 and self.position_uwb_left.x < self.position_uwb_right.x):
            # The front of the Agrobot Gantry is in the north direction
            orientation = 'north'
        elif(abs(self.position_uwb_left.x - self.position_uwb_right.x) > 0.5 and self.position_uwb_left.x > self.position_uwb_right.x):
            # The front of the Agrobot Gantry is in the south direction
            orientation = 'south'
        elif(abs(self.position_uwb_left.y - self.position_uwb_right.y) > 0.5 and self.position_uwb_left.y < self.position_uwb_right.y):
            # The front of the Agrobot Gantry is in the west direction
            orientation = 'west'
        elif(abs(self.position_uwb_left.y - self.position_uwb_right.y) > 0.5 and self.position_uwb_left.y > self.position_uwb_right.y):
            # The front of the Agrobot Gantry is in the east direction
            orientation = 'east'

        return orientation

    # Publish the arduino command
    def publish_arduino_command(self, data):
        # 0 = stop
        # 1 = drive forward
        # 2 = drive backward
        # 3 = turn left
        # 4 = turn right
        self.arduino_drive_command_publisher.publish(data)

    # Subscriber callback when a object is detected that is within 30 cm of the Agrobot Gantry (distance is hardcoded in Arduino)
    def object_detected_callback(self, message):
        self.object_detected = message.data

    # Subscriber callback for the position of the left UWB module
    def uwb_left_callback(self, message):
        self.position_uwb_left = message.pose.position

    # Subscriber callback for the position of the right UWB module
    def uwb_right_callback(self, message):
        self.position_uwb_right = message.pose.position

    # Subscriber callback to drive to the correct position
    def drive_to_position_callback(self, message):
        target_position = Point()

        # Select the correct position object according to the input string
        if(message.data == 'P1'):
            target_position = self.POSITION_P1
        elif(message.data == 'P2'):
            target_position = self.POSITION_P2
        elif(message.data == 'P3'):
            target_position = self.POSITION_P3
        elif(message.data == 'P4'):
            target_position = self.POSITION_P4
        elif(message.data == 'B11'):
            target_position = self.POSITION_B11
        elif(message.data == 'B12'):
            target_position = self.POSITION_B12
        elif(message.data == 'B13'):
            target_position = self.POSITION_B13
        elif(message.data == 'B21'):
            target_position = self.POSITION_B21
        elif(message.data == 'B22'):
            target_position = self.POSITION_B22
        elif(message.data == 'B23'):
            target_position = self.POSITION_B23

        # Check if robot is in the same row as the wanted position (within 0.2 meter)
        if(abs(self.position_agrobot.x - target_position.x) < 0.2):
            # Agrobot Gantry is in same row as target position
            self.drive_forward_to_target(target_position)
        elif(self.position_agrobot.x - target_position.x > 0.2):
            # The Agrobot Gantry is in the right row and the target position in the left row
            self.drive_forward_to_target(self.POSITION_P2)
            self.turn_left()
            self.drive_forward_to_target(self.POSITION_P3)
            self.turn_left()
            self.drive_forward_to_target(target_position)
        elif(self.position_agrobot.x - target_position.x < -0.2):
            # The Agrobot Gantry is in the left row and the target position in the right row
            self.drive_forward_to_target(self.POSITION_P3)
            self.turn_right()
            self.drive_forward_to_target(self.POSITION_P2)
            self.turn_right()
            self.drive_forward_to_target(target_position)


# Initialise node and call the program
if __name__ == '__main__':
    rospy.init_node('drive_node')
    Drive()
    rospy.spin()