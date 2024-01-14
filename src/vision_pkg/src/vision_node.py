#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import Point

class CameraNode:

    def __init__(self, video_source=0):
        self.capture = cv2.VideoCapture(video_source)

        if not self.capture.isOpened():
            raise ValueError("Unable to open the camera. Check if the video source is valid.")

        # Initial lower and upper bounds for detecting green color
        self.lower_green = np.array([35, 70, 70])
        self.upper_green = np.array([100, 255, 255])

        # Initialise subscriber
        rospy.Subscriber('/start_vision', Empty, self.start_vision_callback)

        # Initialise publisher
        self.vision_coordinates_publisher = rospy.Publisher('/vision_coordinates', Point, queue_size=10)

    def run(self):
        try:
            ret, frame = self.capture.read()
            if ret:
                height, width, _ = frame.shape

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)

                # Find contours in the mask
                contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Calculate centroids of contours and draw them
                centroids = []
                for contour in contours:
                    # Calculate centroid using moments
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        centroids.append((cX, cY))
                        cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)

                # Display information about detected centroids
                if centroids:
                    avg_cX = int(sum([c[0] for c in centroids]) / len(centroids))
                    avg_cY = int(sum([c[1] for c in centroids]) / len(centroids))
                    rel_posX = avg_cX - (width // 2)
                    rel_posY = avg_cY - (height // 2)
                    print(f"Average Position: ({avg_cX}, {avg_cY})")
                    print(f"Relative Position to Middle: ({rel_posX}, {rel_posY})")

                    # Publish position as ROS message
                    position = Point()
                    position.x = rel_posX
                    position.y = rel_posY
                    self.vision_coordinates_publisher.publish(position)

                '''
                # De volgende regels zijn enkel als de uitkomst gevisualiseerd moet worden
                cv2.line(frame, (width // 2, 0), (width // 2, height), (255, 255, 255), 1)
                cv2.line(frame, (0, height // 2), (width, height // 2), (255, 255, 255), 1)

                cv2.imshow('Camera Feed', frame)
                cv2.imshow('Green Mask', mask_green)
                '''
            else:
                print("Error reading frame.")
        except Exception as e:
            print(f"An error occurred: {e}")

    def release_camera(self):
        self.capture.release()
        cv2.destroyAllWindows()

    def start_vision_callback(self, msg):
        try:
            self.run()
        except ValueError as e:
            print(e)
        except KeyboardInterrupt:
            pass
        finally:
            self.release_camera()

if __name__ == '__main__':
    rospy.init_node('vision_node')
    CameraNode(0)
    rospy.spin()
