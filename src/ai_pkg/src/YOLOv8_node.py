#!/usr/bin/env python3.8

import cv2
import rospy
from std_msgs.msg import String, Empty, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import supervision as sv


# "/home/jetson/Documents/agrobot_ws/src/agrobot_yolov8/src/nano_model/best.pt"
# "/home/jetson/Documents/agrobot_ws/src/agrobot_yolov8/src/nano_model/best.onnx"
# "/home/jetson/Documents/agrobot_ws/src/agrobot_yolov8/src/nano_model/best.engine"

class YOLOv8ROS:
    def __init__(self, webcam_resolution=(1280, 720), model_path=r"/home/jetson/Documents/agrobot_ws/src/agrobot_yolov8/src/nano_model/best.pt"):
        '''
        self.frame_width, self.frame_height = webcam_resolution
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if not self.cap.isOpened():
            rospy.logerr("Cannot open camera")
            exit()
        '''

        self.model = YOLO(model_path, task='detect')
        self.box_annotator = sv.BoxAnnotator(thickness=2, text_thickness=2, text_scale=1)
        self.prev_time = rospy.Time.now()

        # Subscribers
        rospy.Subscriber('/start_yolov8', Empty, self.start_yolo_callback)

        # Publishers
        self.image_pub = rospy.Publisher('/image_with_detections', Image, queue_size=10)
        self.yolo_detection_publisher = rospy.Publisher('/vision_yolov8_result', Int32, queue_size=10)
        self.bridge = CvBridge()

        #self.timer = rospy.Timer(rospy.Duration(0.03), self.detect_objects)  # Run detection every 0.03 seconds

    def detect_objects(self):
            webcam_resolution=(1280, 720)
            self.frame_width, self.frame_height = webcam_resolution
            self.cap = cv2.VideoCapture(3)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            if not self.cap.isOpened():
                rospy.logerr("Cannot open camera")
                exit()

            ret, frame = self.cap.read()
            results = self.model(frame, agnostic_nms=True, conf=0.65)[0]
            detections = sv.Detections.from_ultralytics(results)

            # Classificaties afdrukken
            for class_id in range(len(detections.class_id)):
                class_name = results.names[detections.class_id[class_id]]
                confidence = detections.confidence[class_id]
                
                if class_name == "radish":
                    self.yolo_detection_publisher.publish(3)
                    print("RADISH")
                elif class_name == "lettuce":
                    self.yolo_detection_publisher.publish(2)
                    print("LETTUCE")
                elif class_name == "carrot":
                    self.yolo_detection_publisher.publish(0)
                    print("CARROT")
                elif class_name == "beetroot":
                    self.yolo_detection_publisher.publish(1)
                    print("BEETROOT") 

            '''
            Allemaal voor testen en niet relevant voor werking. Kost dan extra rekenkracht
            labels = [
                f"{results.names[detections.class_id[class_id]]} {detections.confidence[class_id]:0.2f}"
                for class_id in range(len(detections.class_id))
            ]

            annotated_image = sv.BoundingBoxAnnotator().annotate(scene=frame, detections=detections)
            annotated_image = sv.LabelAnnotator().annotate(scene=annotated_image, detections=detections, labels=labels)

            # Update van de FPS-overlay
            current_time = rospy.Time.now()
            time_diff = current_time - self.prev_time
            fps = 1.0 / time_diff.to_sec() if time_diff.to_sec() > 0 else 0.0  # Bereken FPS

            cv2.putText(annotated_image, f'FPS: {fps:.0f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Convert OpenCV image to ROS message
            image_message = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            self.image_pub.publish(image_message)

            cv2.imshow("yolov8", annotated_image)  
            
            self.prev_time = current_time  # Update van de vorige tijd voor de volgende frame

            if cv2.waitKey(30) == 27:  # press esc
                self.cap.release()
                cv2.destroyAllWindows()
            '''

            self.cap.release()

    def start_yolo_callback(self, msg):
        self.detect_objects()

if __name__ == "__main__":
    rospy.init_node('yolov8_node')
    rospy.loginfo("YOLOv8 ROS node started!")
    detector = YOLOv8ROS()
    rospy.spin()  # Keeps the node running until shutdown
