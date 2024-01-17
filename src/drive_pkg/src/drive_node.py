#! /usr/bin/env python3

# =====================
# Author: Cas Damen
# Created on: 31-10-2023
# Description: Class to control the drive and steering of the Agrobot Gantry
# =====================

import rospy
from std_msgs.msg import Bool, Int8, String, Empty
from geometry_msgs.msg import PoseStamped, Point

class Drive(object):

    # Constructor of the Drive class
    def __init__(self):
        # Properties for the locations in the playing field (distances in meters)
        # Those properties are constants and should not be changed during the program
        # A visible presentation of all the positions can be found in drive_pkg/docs/Agrobot Gantry waypoints.png
        # Some positions have offsets which are determed during testing
        self.POSITION_P1 = Point()
        self.POSITION_P1.x = 2.5 - 0.15
        self.POSITION_P1.y = 1.25
        self.POSITION_P2 = Point()
        self.POSITION_P2.x = 2.5 - 0.15
        self.POSITION_P2.y = 5.75
        self.POSITION_P3 = Point()
        self.POSITION_P3.x = 1.0 + 0.1
        self.POSITION_P3.y = 5.75
        self.POSITION_P4 = Point()
        self.POSITION_P4.x = 1.0 + 0.1
        self.POSITION_P4.y = 1.25

        self.POSITION_B11 = Point()
        self.POSITION_B11.x = 2.5
        self.POSITION_B11.y = 2.5 + 0.20
        self.POSITION_B11r2 = Point()
        self.POSITION_B11r2.x = 2.5
        self.POSITION_B11r2.y = 2.5 + 0.20 + 0.3
        self.POSITION_B11r3 = Point()
        self.POSITION_B11r3.x = 2.5
        self.POSITION_B11r3.y = 2.5 + 0.20 + 0.6
        self.POSITION_B12 = Point()
        self.POSITION_B12.x = 2.5
        self.POSITION_B12.y = 3.5 + 0.20
        self.POSITION_B12r2 = Point()
        self.POSITION_B12r2.x = 2.5
        self.POSITION_B12r2.y = 3.5 + 0.20 + 0.3
        self.POSITION_B12r3 = Point()
        self.POSITION_B12r3.x = 2.5
        self.POSITION_B12r3.y = 3.5 + 0.20 + 0.6
        self.POSITION_B13 = Point()
        self.POSITION_B13.x = 2.5
        self.POSITION_B13.y = 4.5 + 0.20

        self.POSITION_B21 = Point()
        self.POSITION_B21.x = 1.0
        self.POSITION_B21.y = 2.5 - 0.40
        self.POSITION_B22 = Point()
        self.POSITION_B22.x = 1.0
        self.POSITION_B22.y = 3.5 - 0.40
        self.POSITION_B22r2 = Point()
        self.POSITION_B22r2.x = 1.0
        self.POSITION_B22r2.y = 3.5 - 0.40 - 0.3
        self.POSITION_B22r3 = Point()
        self.POSITION_B22r3.x = 1.0
        self.POSITION_B22r3.y = 3.5 - 0.40 - 0.6
        self.POSITION_B23 = Point()
        self.POSITION_B23.x = 1.0
        self.POSITION_B23.y = 4.5 - 0.40
        self.POSITION_B23r2 = Point()
        self.POSITION_B23r2.x = 1.0
        self.POSITION_B23r2.y = 4.5 - 0.40 - 0.3
        self.POSITION_B23r3 = Point()
        self.POSITION_B23r3.x = 1.0
        self.POSITION_B23r3.y = 4.5 - 0.40 - 0.6

        # Properties for subscribers callbacks
        self.object_detected = False
        self.arduino_steering_state = 0

        # Properties for positions of the UWB modules
        self.position_uwb_left = Point()
        self.position_uwb_right = Point()

        # Properties for the center position of the Agrobot Gantry
        self.position_agrobot = Point()
        self.position_agrobot.x = (self.position_uwb_left.x + self.position_uwb_right.x) / 2
        self.position_agrobot.y = (self.position_uwb_left.y + self.position_uwb_right.y) / 2

        # Initialise subscribers
        rospy.Subscriber('/agrobot_object_detection/object_detected', Bool, self.object_detected_callback)
        rospy.Subscriber('/agrobot_steering/arduino_state', Int8, self.arduino_steering_state_callback)
        rospy.Subscriber('/dwm1001/tag/tagLeft/position', PoseStamped, self.uwb_left_callback)
        rospy.Subscriber('/dwm1001/tag/tagRight/position', PoseStamped, self.uwb_right_callback)
        rospy.Subscriber('/drive_to_position', String, self.drive_to_position_callback)
        rospy.Subscriber('/initialise_wheels', Empty, self.initialise_wheels_callback)

        # Initialise publishers
        self.arduino_drive_command_publisher = rospy.Publisher('/agrobot_drive/arduino_command', Int8, queue_size=10)
        self.arduino_steering_command_publisher = rospy.Publisher('/agrobot_steering/arduino_command', Int8, queue_size=10)
        self.initialise_wheels_done_publisher = rospy.Publisher('/initialise_wheels_done', Empty, queue_size=10)
        self.driving_done_publisher = rospy.Publisher('/driving_done', Empty, queue_size=10)

    # Drive the Agrobot Gantry forward untill the target position is reached
    def drive_forward_to_target(self, target_position):
        target_reached = False
        orientation = self.get_orientation()
        direction = 'forward'

        # Publish in the Arduino command topic to start driving forward
        if(orientation == 'north'):
            if(target_position.y <=  self.position_uwb_left.y):
                self.publish_arduino_drive_command(2)
                direction = 'backward'
            else:
                self.publish_arduino_drive_command(1)
                direction = 'forward'
        elif(orientation == 'south'):
            if(target_position.y >=  self.position_uwb_left.y):
                self.publish_arduino_drive_command(2)
                direction = 'backward'
            else:
                self.publish_arduino_drive_command(1)
                direction = 'forward'
        elif(orientation == 'east'):
            if(target_position.x <=  self.position_uwb_left.x):
                self.publish_arduino_drive_command(2)
                direction = 'backward'
            else:
                self.publish_arduino_drive_command(1)
                direction = 'forward'
        elif(orientation == 'west'):
            if(target_position.x >=  self.position_uwb_left.x):
                self.publish_arduino_drive_command(2)
                direction = 'backward'
            else:
                self.publish_arduino_drive_command(1)
                direction = 'forward'

        # Drive untill the target position is reached
        #
        #
        # VERGEET NIET HET VOLGENDE TERUG TE PLAATSEN: and not self.object_detected
        while(not target_reached and not rospy.is_shutdown()):
            # Update the position of the Agrobot Gantry
            #self.position_agrobot.x = (self.position_uwb_left.x + self.position_uwb_right.x) / 2
            #self.position_agrobot.y = (self.position_uwb_left.y + self.position_uwb_right.y) / 2
            self.position_agrobot.x = self.position_uwb_left.x
            self.position_agrobot.y = self.position_uwb_left.y

            if(direction == 'forward'):
                if(orientation == 'north'):
                    target_reached = self.position_agrobot.y >= target_position.y
                elif(orientation == 'south'):
                    target_reached = self.position_agrobot.y <= target_position.y
                elif(orientation == 'west'):
                    target_reached = self.position_agrobot.x <= target_position.x
                elif(orientation == 'east'):
                    target_reached = self.position_agrobot.x >= target_position.x
            elif(direction == 'backward'):
                if(orientation == 'north'):
                    target_reached = self.position_agrobot.y <= target_position.y
                elif(orientation == 'south'):
                    target_reached = self.position_agrobot.y >= target_position.y
                elif(orientation == 'west'):
                    target_reached = self.position_agrobot.x >= target_position.x
                elif(orientation == 'east'):
                    target_reached = self.position_agrobot.x <= target_position.x

        # Publish in the Arduino command topic to stop the Agrobot Gantry
        self.publish_arduino_drive_command(0)

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

    # Turn the Agrobot Gantry 90 degrees
    def turn(self, turn_direction, turn_orientation):
        if(turn_direction == 'left'):
            turn_command = 3
        elif(turn_direction == 'right'):
            turn_command = 4

        # Make the wheels turn to the steering positiong
        self.publish_arduino_steering_command(3)
        rospy.sleep(2)

        # Wait untill the steering is done
        #
        # TERUG ZETTEN: and not self.object_detected
        while(self.arduino_steering_state > 0 and not rospy.is_shutdown()):
            # Wait till the Arduino sends a command it is done with turning the wheels
            # The Arduino can do this task by itself, so a wait of 1 second is fine
            rospy.sleep(1)

        # Turn the robot untill it is rotated 90 degrees
        self.publish_arduino_drive_command(turn_command)
        position_reached = False
        #
        # TERUG ZETTEN: and not self.object_detected
        while(not position_reached and not rospy.is_shutdown()):
            if(turn_orientation == 'HorizontalToVertical' and abs(self.position_uwb_left.x - self.position_uwb_right.x) < 0.01):
                position_reached = True
            elif(turn_orientation == 'VerticalToHorizontal' and abs(self.position_uwb_left.y - self.position_uwb_right.y) < 0.01):
                position_reached = True

        self.publish_arduino_drive_command(0)
        rospy.sleep(1)

        # Make the wheels turn to the straight position
        self.publish_arduino_steering_command(2)
        rospy.sleep(2)

        # Wait untill the steering is done
        #
        # TERUG ZETTEN: and not self.object_detected
        while(self.arduino_steering_state > 0 and not rospy.is_shutdown()):
            # Wait till the Arduino sends a command it is done with turning the wheels
            rospy.sleep(1)

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

    # Publish the arduino drive command
    def publish_arduino_drive_command(self, data):
        # 0 = idle / stop
        # 1 = drive forward
        # 2 = drive backward
        # 3 = turn left
        # 4 = turn right
        self.arduino_drive_command_publisher.publish(data)

    # Publish the arduino steering command
    def publish_arduino_steering_command(self, data):
        # 0 = idle
        # 1 = initialise
        # 2 = turn wheels to straight position
        # 3 = turn wheels to turn position
        self.arduino_steering_command_publisher.publish(data)

    # Subscriber callback for initialising the wheels
    def initialise_wheels_callback(self, message):
        # Publish command to Arduino steering
        self.publish_arduino_steering_command(1)
        rospy.sleep(2)

        # Wait till the Arduino sends a command it is done with turning the wheels
        # The Arduino can do this task by itself, so a wait of 1 second is fine
        while(self.arduino_steering_state > 0 and not rospy.is_shutdown()):
            rospy.sleep(1)

        msg = Empty()
        self.initialise_wheels_done_publisher.publish(msg)

    # Subscriber callback when a object is detected that is within 30 cm of the Agrobot Gantry (distance is hardcoded in Arduino)
    def object_detected_callback(self, message):
        self.object_detected = message.data

    # Subscriber callback for the state of the Arduino that is responsible for the steering
    def arduino_steering_state_callback(self, message):
        self.arduino_steering_state = message.data

    # Subscriber callback for the position of the left UWB module
    def uwb_left_callback(self, message):
        self.position_uwb_left = message.pose.position

    # Subscriber callback for the position of the right UWB module
    def uwb_right_callback(self, message):
        self.position_uwb_right = message.pose.position

    # Subscriber callback to drive to the correct position
    def drive_to_position_callback(self, message):
        if(message.data == 'NEXT'):
            # Drive to the next row
            self.drive_to_next_row()
        else:
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
            elif(message.data == 'B11r2'):
                target_position = self.POSITION_B11r2
            elif(message.data == 'B11r3'):
                target_position = self.POSITION_B11r3
            elif(message.data == 'B12'):
                target_position = self.POSITION_B12
            elif(message.data == 'B12r2'):
                target_position = self.POSITION_B12r2
            elif(message.data == 'B12r3'):
                target_position = self.POSITION_B12r3
            elif(message.data == 'B13'):
                target_position = self.POSITION_B13
            elif(message.data == 'B21'):
                target_position = self.POSITION_B21
            elif(message.data == 'B22'):
                target_position = self.POSITION_B22
            elif(message.data == 'B22r2'):
                target_position = self.POSITION_B22r2
            elif(message.data == 'B22r3'):
                target_position = self.POSITION_B22r3
            elif(message.data == 'B23'):
                target_position = self.POSITION_B23
            elif(message.data == 'B23r2'):
                target_position = self.POSITION_B23r2
            elif(message.data == 'B23r3'):
                target_position = self.POSITION_B23r3

            # Update the position of the Agrobot Gantry
            self.position_agrobot.x = (self.position_uwb_left.x + self.position_uwb_right.x) / 2
            self.position_agrobot.y = (self.position_uwb_left.y + self.position_uwb_right.y) / 2
            
            # Check if robot is in the same row as the wanted position (within 0.2 meter)
            if(abs(self.position_agrobot.x - target_position.x) < 0.5):
                print("Target is in zelfde rij")
                # Agrobot Gantry is in same row as target position
                self.drive_forward_to_target(target_position)
            elif(self.position_agrobot.x - target_position.x > 0.5):
                print("Target is in linker rij, agrobot is in rechter rij")
                # The Agrobot Gantry is in the right row and the target position in the left row
                self.drive_forward_to_target(self.POSITION_P2)
                self.turn('left', 'HorizontalToVertical')
                self.drive_forward_to_target(self.POSITION_P3)
                self.turn('left', 'VerticalToHorizontal')
                self.drive_forward_to_target(target_position)
            elif(self.position_agrobot.x - target_position.x < -0.5):
                print("Target is in rechter rij, agrobot is in linker rij")
                # The Agrobot Gantry is in the left row and the target position in the right row
                self.drive_forward_to_target(self.POSITION_P3)
                self.turn('right', 'HorizontalToVertical')
                self.drive_forward_to_target(self.POSITION_P2)
                self.turn('right', 'VerticalToHorizontal')
                self.drive_forward_to_target(target_position)

        # Publish empty message so the main program knows the target is reached
        msg = Empty()
        self.driving_done_publisher.publish(msg)
        
# Initialise node and call the program
if __name__ == '__main__':
    rospy.init_node('drive_node')
    Drive()
    rospy.spin()