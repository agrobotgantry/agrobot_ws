#! /usr/bin/env python3

# =====================
# Author: Cas Damen
# Created on: 09-01-2024
# Description: Class to control the main program of the Agrobot Gantry
# =====================

import rospy
from std_msgs.msg import Bool, Int8, String, Empty

class Main(object):

    # Constructor of the Main class
    def __init__(self):
        # Units initialise checks
        self.drive_initialised = False

        # The number of the last vegetable that is harvested
        self.last_vegetable_location = 0

        # The storage is full
        self.storage_full = False

        # Initialise subscribers
        rospy.Subscriber('/agrobot_start', Empty, self.agrobot_start_callback)
        rospy.Subscriber('/driving_done', Empty, self.driving_done_callback)
        rospy.Subscriber('/initialise_wheels_done', Empty, self.initialise_wheels_done_callback)
        rospy.Subscriber('/gantry_done', Empty, self.gantry_done)
        rospy.Subscriber('/storage_is_full', Empty, self.storage_is_full)

        # Initialise publishers
        self.initialise_wheels_publisher = rospy.Publisher('/initialise_wheels', Empty, queue_size=10)
        self.drive_location_publisher = rospy.Publisher('/drive_to_position', String, queue_size=10)
        self.start_gantry_publisher = rospy.Publisher('/start_gantry', String, queue_size=10)
        self.start_storage_publisher = rospy.Publisher('agrobot_storage/arduino_command', Int8, queue_size=10)

    # Drive to the next position
    def drive_to_next_position(self):
        # Go to the next vegatable location
        self.last_vegetable_location += 1

        print('Current vegetable: ' + str(self.last_vegetable_location))

        # Determ the correct position
        if(self.last_vegetable_location == 1):
            self.drive_location_publisher.publish('B11')
        elif(self.last_vegetable_location == 4 or self.last_vegetable_location == 7):
            self.drive_location_publisher.publish('NEXT')
        elif(self.last_vegetable_location == 10):
            self.drive_location_publisher.publish('B12')
        elif(self.last_vegetable_location == 13 or self.last_vegetable_location == 16):
            self.drive_location_publisher.publish('NEXT')
        elif(self.last_vegetable_location == 19):
            self.drive_location_publisher.publish('B23')
        elif(self.last_vegetable_location == 22 or self.last_vegetable_location == 25):
            self.drive_location_publisher.publish('NEXT')
        elif(self.last_vegetable_location == 28):
            self.drive_location_publisher.publish('B22')
        elif(self.last_vegetable_location == 31 or self.last_vegetable_location == 34):
            self.drive_location_publisher.publish('NEXT')

    # Start the gantry to harvest a vegatable
    def start_gantry(self):
        if(self.last_vegetable_location == 1 or self.last_vegetable_location == 4 or self.last_vegetable_location == 7):
            self.start_gantry_publisher.publish('left')
        elif(self.last_vegetable_location == 2 or self.last_vegetable_location == 5 or self.last_vegetable_location == 8):
            self.start_gantry_publisher.publish('middle')
        elif(self.last_vegetable_location == 3 or self.last_vegetable_location == 6 or self.last_vegetable_location == 9):
            self.start_gantry_publisher.publish('right')
        elif(self.last_vegetable_location == 10 or self.last_vegetable_location == 13 or self.last_vegetable_location == 16):
            self.start_gantry_publisher.publish('left')
        elif(self.last_vegetable_location == 11 or self.last_vegetable_location == 14 or self.last_vegetable_location == 17):
            self.start_gantry_publisher.publish('middle')
        elif(self.last_vegetable_location == 12 or self.last_vegetable_location == 15 or self.last_vegetable_location == 18):
            self.start_gantry_publisher.publish('right')
        elif(self.last_vegetable_location == 19 or self.last_vegetable_location == 22 or self.last_vegetable_location == 25):
            self.start_gantry_publisher.publish('left')
        elif(self.last_vegetable_location == 20 or self.last_vegetable_location == 23 or self.last_vegetable_location == 26):
            self.start_gantry_publisher.publish('middle')
        elif(self.last_vegetable_location == 21 or self.last_vegetable_location == 24 or self.last_vegetable_location == 27):
            self.start_gantry_publisher.publish('right')
        elif(self.last_vegetable_location == 28 or self.last_vegetable_location == 31 or self.last_vegetable_location == 34):
            self.start_gantry_publisher.publish('left')
        elif(self.last_vegetable_location == 29 or self.last_vegetable_location == 32 or self.last_vegetable_location == 35):
            self.start_gantry_publisher.publish('middle')
        elif(self.last_vegetable_location == 30 or self.last_vegetable_location == 33 or self.last_vegetable_location == 36):
            self.start_gantry_publisher.publish('right')

    # Start the agrobot program
    def agrobot_start_callback(self, data):
        print('agrobot gestart')

        # Initialise bakken
        self.start_storage_publisher.publish(1)
        rospy.sleep(1)

        '''
        # Initialise the wheels
        msg = Empty()
        self.initialise_wheels_publisher.publish(msg)
        rospy.sleep(1)

        # Wait till all units are initialised
        while(not self.drive_initialised):
            rospy.sleep(1)
        '''

        # Go to the first position
        self.drive_to_next_position()

    # Subscriber callback initialise wheels done
    def initialise_wheels_done_callback(self, data):
        self.drive_initialised = True
        print('Wielen geinitialiseerd')

    # Subscriber callback driving done
    def driving_done_callback(self, data):
        print('rijden klaar')

        if(self.storage_full):
            # Empty storage
            self.start_storage_publisher.publish(2)
            rospy.sleep(5)
            self.start_storage_publisher.publish(1)
            self.storage_full = False

            # Drive back to the last location
            if(self.last_vegetable_location == 1 or self.last_vegetable_location == 2 or self.last_vegetable_location == 3):
                self.drive_location_publisher.publish('B11')
            elif(self.last_vegetable_location == 4 or self.last_vegetable_location == 5 or self.last_vegetable_location == 6):
                self.drive_location_publisher.publish('B11r2')
            elif(self.last_vegetable_location == 7 or self.last_vegetable_location == 8 or self.last_vegetable_location == 9):
                self.drive_location_publisher.publish('B11r3')
            elif(self.last_vegetable_location == 10 or self.last_vegetable_location == 11 or self.last_vegetable_location == 12):
                self.drive_location_publisher.publish('B12')
            elif(self.last_vegetable_location == 13 or self.last_vegetable_location == 14 or self.last_vegetable_location == 15):
                self.drive_location_publisher.publish('B12r2')
            elif(self.last_vegetable_location == 16 or self.last_vegetable_location == 17 or self.last_vegetable_location == 18):
                self.drive_location_publisher.publish('B11r3')
            elif(self.last_vegetable_location == 19 or self.last_vegetable_location == 20 or self.last_vegetable_location == 21):
                self.drive_location_publisher.publish('B23')
            elif(self.last_vegetable_location == 22 or self.last_vegetable_location == 23 or self.last_vegetable_location == 24):
                self.drive_location_publisher.publish('B23r2')
            elif(self.last_vegetable_location == 25 or self.last_vegetable_location == 26 or self.last_vegetable_location == 27):
                self.drive_location_publisher.publish('B23r3')
            elif(self.last_vegetable_location == 28 or self.last_vegetable_location == 29 or self.last_vegetable_location == 30):
                self.drive_location_publisher.publish('B22')
            elif(self.last_vegetable_location == 31 or self.last_vegetable_location == 32 or self.last_vegetable_location == 33):
                self.drive_location_publisher.publish('B22r2')
            elif(self.last_vegetable_location == 34 or self.last_vegetable_location == 35 or self.last_vegetable_location == 36):
                self.drive_location_publisher.publish('B22r3')
        else:
            # Start gantry
            self.start_gantry()

    # Subscriber callback gantry done
    def gantry_done(self, data):
        print('gantry klaar')
        
        # Check if the storage is full
        if(self.storage_full):           
            self.drive_location_publisher.publish('P1')
            print('bakken legen')
        else:
            self.drive_to_next_position()

    # Subscriber callback storage done
    def storage_is_full(self, data):
        self.storage_full = True

# Initialise node and call the program
if __name__ == '__main__':
    rospy.init_node('main_node')
    Main()
    rospy.spin()