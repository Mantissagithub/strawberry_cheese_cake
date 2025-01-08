
#!/usr/bin/env python3
# for OpenCV
import cv2
import numpy as np
import pyrealsense2 as rs
import sys
import os
import supervision as sv
from ultralytics import YOLO
import math
from sensor_msgs.msg import Image
import cv_bridge
from cv_bridge import CvBridge

# for realsense
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
#from miscellaneous import *

# For ROS
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32
from time import time
import torch

# Constants initialisations for OpenCV
#dc = DepthCamera()
# dc = cv2.VideoCapture(0)

# Parameters for denoising and Canny edge detection
median_blur_ksize = 7
gaussian_blur_ksize = (9, 9)
canny_threshold1 = 50
canny_threshold2 = 150

device = 'cuda' if torch.cuda.is_available() else 'cpu'

# for better arrow tracking
class EuclideanDistTracker:
    def __init__(self):
        self.center_points = {}
        self.id_count = 0

    # this function is to update the arrows
    def update(self, objects_rect):
        objects_bbs_ids = []
        for rect in objects_rect:
            x, y, w, h, contour, direction = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2
            centroid = (cx, cy)

            same_object_detected = False
            for id, pt in self.center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                if dist < 15:
                    self.center_points[id] = (cx, cy)
                    objects_bbs_ids.append(
                        [x, y, w, h, id, contour, centroid, direction]
                    )
                    same_object_detected = True
                    break

            if not same_object_detected:
                self.center_points[self.id_count] = (cx, cy)
                objects_bbs_ids.append(
                    [x, y, w, h, self.id_count, contour, centroid, direction]
                )
                self.id_count += 1

        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            object_id = obj_bb_id[4]
            center = self.center_points[object_id]
            new_center_points[object_id] = center

        self.center_points = new_center_points.copy()
        return objects_bbs_ids


class DirectionPublisher(Node):
    def __init__(self):
        super().__init__("direction_publisher")
        # initialising publishers and creating a timer to call send_cmd_vel function
        self.direction_publisher = self.create_publisher(String, "direction", 10)
        self.distance_threshold = self.create_publisher(Int32, "distance", 10)
        self.publisher_ = self.create_publisher(String, "/stop_command", 10)
        self.align_publisher = self.create_publisher(Twist, "/align_publisher", 10)
        #self.create_timer(1, self.send_cmd_vel)
        self.get_logger().info("Direction Publisher node Chalu")
        # getting the frames from the subscribing to image raw cv2_bridge
        self.color_subscription = self.create_subscription(
        Image, 
        '/camera/camera/color/image_raw', 
        self.color_callback, 
        10
        )
        self.depth_subscription = self.create_subscription(
        Image, 
        '/camera/camera/depth/image_rect_raw', 
        self.depth_callback, 
        10
        )
        self.bridge = CvBridge()
        self.d_frame=None
        self.frame=None
        # initialising some important variables
        self.prev_direction = None
        self.distance_published = False
        self.direction_published = False
        self.stop_published = False
        self.time_delay = 0.0
        self.last_distance_publish_time = None
        self.model = YOLO("/home/pradheep/Documents/strawberry_cheese_cake/irc_autonomous_ws/src/autonomous_stack/autonomous_stack/best.pt											")  # this is the path to the weight file
        self.bounding_box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.cone_class_id = 0
        self.confidence_threshold = 0.875
        self.FOCAL_LENGTH = 75
        self.REAL_ARROW_WIDTH = 300
        self.d_frame=None
        # initilising messages to send in topic
        self.vel_msg = Twist()
        self.align_vel_msg = Twist()
        self.latest_color_msg = None
        self.latest_depth_msg = None
    
    def color_callback(self, color_msg):
        self.frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')

        if self.d_frame is not None:
            self.send_cmd_vel(self,self.frame, self.d_frame)
    def depth_callback(self, depth_msg):
        self.d_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if self.d_frame is not None:
            self.send_cmd_vel(self,self.latest_color_msg, self.latest_depth_msg)
    def arrow_distance_estimation(self, valid, d_frame, cx, cy):
        prev_dist = 0.0
        depth_value = d_frame[cy, cx]
        distance = depth_value / 10
        if distance == 0.0:
            distance = prev_dist
        else:
            prev_dist = distance
        return distance
    

    def control_turtlebot(self, contour, relative_position, distance):
        # Clear velocity message before setting new values
        self.align_vel_msg.linear.x = 0.0
        self.align_vel_msg.angular.z = 0.0

        # Set angular velocity based on relative position (left/right)
        if distance > 70:
            if relative_position[0] < -50:  # Move left
                self.align_vel_msg.angular.z = -1.0
                self.get_logger().info(f"{self.align_vel_msg}")
                self.align_publisher.publish(self.align_vel_msg)
            elif relative_position[0] > 50:  # Move right
                self.align_vel_msg.angular.z = 1.0
                self.get_logger().info(f"{self.align_vel_msg}")
                self.align_publisher.publish(self.align_vel_msg)
            else:
                self.align_vel_msg.angular.z = 0.0
                self.align_vel_msg.linear.x = 1.0
                self.get_logger().info(f"{self.align_vel_msg}")
                self.align_publisher.publish(self.align_vel_msg)
        else:
            self.align_vel_msg.linear.x = 0.0
            self.align_vel_msg.angular.z = 0.0
            # Publish the velocity command

    def calculate_angles(self, approx):
        def angle(pt1, pt2, pt3):
            v1 = np.array(pt1) - np.array(pt2)
            v2 = np.array(pt3) - np.array(pt2)
            cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            cosine_angle = np.clip(cosine_angle, -1.0, 1.0)
            return np.degrees(np.arccos(cosine_angle))

        angles = []
        for i in range(len(approx)):
            pt1 = approx[i - 1][0]
            pt2 = approx[i][0]
            pt3 = approx[(i + 1) % len(approx)][0]
            angles.append(angle(pt1, pt2, pt3))
        return angles

    def send_cmd_vel(self, frame,d_frame):

        msg1 = String()
        msg2 = Int32()
        tracker = EuclideanDistTracker()
        # msg1 = String()
        # msg2 = Int32()
        # tracker = EuclideanDistTracker()
        # while True:
        # for img_msg to cv2
        frame = frame
        d_frame = d_frame
        ret=frame is None and d_frame is None
        cv2.imshow("ROS2 Camera Frame", frame)
        cv2.waitKey(1)

    # Rest of the processing logic from send_cmd_vel
        msg1 = String()
        msg2 = Int32()
        tracker = EuclideanDistTracker()
    
            # for RealSense
            #ret, d_frame, frame = dc.get_frame()
            # for img_msg to cv2
            # ret, frame = dc.read()
        frame_height, frame_width, _ = frame.shape
        frame_center_x = frame_width // 2
            #if not ret:
        if frame is None and d_frame is None:
                print("Error: Could not read frame from webcam.")
                return

            # Apply median blur (stronger than Gaussian for denoising)
        frame_denoised = cv2.medianBlur(frame, median_blur_ksize)

            # Apply Gaussian blur
        frame_denoised = cv2.GaussianBlur(frame_denoised, gaussian_blur_ksize, 0)

            # Convert to grayscale
        gray = cv2.cvtColor(frame_denoised, cv2.COLOR_BGR2GRAY)

            # Apply adaptive thresholding
        binary = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
            )

            # Detect edges using Canny
        edges = cv2.Canny(binary, canny_threshold1, canny_threshold2)

            # Find contours
        contours, _ = cv2.findContours(
                edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

        detected_arrows = []
        for contour in contours:
                # Filter small contours
                if cv2.contourArea(contour) < 1000:  # Adjust threshold if needed
                    continue

                # Approximate the contour to a polygon
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # Check if it's arrow-like (with 7 vertices)
                x, y, w, h = cv2.boundingRect(contour)
                if len(approx) == 7:
                    angles = self.calculate_angles(approx)
                    tip_angle1, tip_angle2, tip_angle3 = angles[0], angles[1], angles[2]
                    base_angle1, base_angle2 = angles[3], angles[4]
                    if (
                        50 <= tip_angle1 <= 160
                        and 50 <= base_angle1 <= 160
                        and 50 <= base_angle2 <= 160
                        and 30 <= tip_angle2 <= 100
                        and 50 <= tip_angle3 <= 160
                    ):
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            centroid = (cx, cy)
                            # Compute the bounding box around the arrow
                            x, y, w, h = cv2.boundingRect(contour)
                            # Store the detected arrow data
                            farthest_point = max(
                                approx, key=lambda p: np.linalg.norm(p[0] - centroid)
                            )
                            tip = tuple(farthest_point[0])
                            # Determine the direction of the arrow
                            if tip[0] < centroid[0]:
                                direction = "Right"
                            else:
                                direction = "Left"
                            detected_arrows.append((x, y, w, h, contour, direction))
                # global boxes_ids
        boxes_ids = tracker.update(detected_arrows)
        distance, prev_dist = 0,0
        for boxes in boxes_ids:
                x, y, w, h, id = boxes[:5]
                # Draw the arrow's contour
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                mid_x = x + w // 2
                mid_y = y + h // 2
                cv2.circle(frame, (mid_x, mid_y), 10, (0, 0, 255), -1)
                direction = str(boxes[-1])
                cv2.putText(
                    frame,
                    direction,
                    (mid_x, mid_y + 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (255, 0, 0),
                    2,
                )
                contour = boxes[5]
                centroid = boxes[6]
                relative_position = (
                    centroid[0] - frame_center_x,
                    centroid[1] - frame_height // 2,
                )
                distance = self.arrow_distance_estimation(
                    ret, d_frame, centroid[0], centroid[1]
                )
                self.control_turtlebot(contour, relative_position, distance)
                # Display the arrow's data
                cv2.putText(
                    frame,
                    f"{distance}mm",
                    (mid_x, mid_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 255),
                    2,
                )
                # for arrow tip
                # cv2.circle(frame, tip, 5, (0, 0, 255), -1)
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

        if not self.distance_published and 50 <= distance <= 70:
                msg2.data = int(distance)
                self.distance_threshold.publish(msg=msg2)
                self.get_logger().info(f" Distance: {distance}")
                self.distance_published = True
                self.last_distance_publish_time = time()

        if (
                self.distance_published
                and not self.direction_published
                and self.last_distance_publish_time
                and time() - self.last_distance_publish_time >= self.time_delay
            ):
                msg1.data = direction
                self.direction_publisher.publish(msg=msg1)
                self.get_logger().info(f"Published direction: {direction}")
                self.direction_published = True

        if distance < 50 or distance > 70:
                self.distance_published = False
                self.direction_published = False
                self.last_distance_publish_time = None

            # else:
            #     buffer.clear()
            # Run YOLO model on the color image
            # results = self.model(frame)[0]
        results = self.model.predict(source=frame, verbose=False)[0]
        detections = sv.Detections.from_ultralytics(results)

            # Filter detections: Only keep cones with high confidence
        cone_detections = detections[
                (detections.confidence > self.confidence_threshold)
                & (detections.class_id == self.cone_class_id)
            ]

        if (
                len(cone_detections) > 0 and not self.stop_published
            ):  # If cones are detected and stop hasn't been published yet
                stop_msg = String()  # Create a String message
                stop_msg.data = "stop"  # Set the message data to "stop"
                self.publisher_.publish(stop_msg)  # Publish the stop message
                self.get_logger().info("Stop command published!")
                self.stop_published = True  # Set flag to true after publishing

            # Annotate and display the image
        annotated_image = self.bounding_box_annotator.annotate(
                scene=frame, detections=cone_detections
            )
        annotated_image = self.label_annotator.annotate(
                scene=annotated_image, detections=cone_detections
            )
            # cv2.imshow('Cone Detection', annotated_image)

            # Display the processed frame
            # fram = cv2.flip(frame, 0)
        cv2.imshow("Detected Arrows", frame)

            # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return

        # dc.release()
        # cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = DirectionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
