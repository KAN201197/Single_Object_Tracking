#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D
from std_msgs.msg import String

class SingleObjectTracking:
    def __init__(self):

        rospy.init_node('object_tracker_node', anonymous=True)

        # Retrieve parameters from the parameter server
        self.groundtruth_path = rospy.get_param("~groundtruth_path")
        self.firsttrack_path = rospy.get_param("~firsttrack_path")

        # Initialize Kalman filter parameters
        self.F = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])  # State Transition Matrix

        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])  # Measurement Matrix

        self.P = np.eye(4) * 1e2  # Covariance matrix initialization
        self.R = np.eye(4) * 1e1  # Measurement noise covariance matrix
        self.Q = np.eye(4) * 1e1  # Process noise covariance matrix

        # read and load the initial state from firsttrack data and ground truth from ground truth data
        self.state = self.read_firsttrack()
        self.ground_truth = self.read_groundtruth()

        self.bridge = CvBridge()

        # declare publisher for tracked object, ground truth and matric number
        self.tracked_object_pub = rospy.Publisher('/tracked_object', Detection2D, queue_size=10)
        self.ground_truth_pub = rospy.Publisher('/ground_truth_objects', Detection2D, queue_size=10)
        self.matric_number_pub = rospy.Publisher('/matric_number', String, queue_size=10)

        # declare publisher for image with bounding box of tracked object
        self.bounded_image_pub = rospy.Publisher('/bounded_image', Image, queue_size=10)

        # declare subscriber to retrive raw image message
        self.image_sub = rospy.Subscriber('/me5413/image_raw', Image, self.image_callback)

        # count use to select corresponding ground truth data for measurement variable
        self.count = 0
    
    # implement function to read groundtruth data
    def read_groundtruth(self):
        with open(self.groundtruth_path, 'r') as file:
            lines = file.readlines()
            ground_coords = [list(map(int,line.strip().split(','))) for line in lines]
        return ground_coords
        
    # implement function to read firstract data
    def read_firsttrack(self):
        with open(self.firsttrack_path, 'r') as file:
            lines = file.readlines()
            template_coords = [list(map(int, line.strip().split(','))) for line in lines][0]
        return template_coords

    # implement function to calculate kalman filter
    def kalman_filter_implementation(self,state, measurement):
        F = np.array(self.F, dtype=np.float32)
        H = np.array(self.H, dtype=np.float32)
        P = np.array(self.P, dtype=np.float32)
        R = np.array(self.R, dtype=np.float32)
        Q = np.array(self.Q, dtype=np.float32)

        # Start with prediction step
        state = F @ state
        P = F @ P @ F.T + Q

        # Update step using measurement from ground truth data
        measurement = np.array(measurement, dtype=np.float32).reshape(-1, 1)
        y = measurement - H @ state
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        state += K @ y
        P = (np.eye(4) - K @ H) @ P

        self.state = state.flatten()

        return state.flatten()
    
    # callback function
    def image_callback(self, msg):

        cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

        ground_truth = np.array(self.ground_truth).reshape(-1,4)
        
        # measurement using selected index of ground truth data
        measurement = ground_truth[self.count]
        self.count += 1
       
        # if count reach 100 (reach final frame) then reset count to 0 to get the first initial measurement from ground truth data
        if self.count == 100 or msg == None:
            self.count = 0

        state = np.array(self.state, dtype=np.float32).reshape(-1,1)

        # implement object tracking using kalman filter algorithm
        predicted_state = self.kalman_filter_implementation(state, measurement)
        self.state = predicted_state

        # Draw bounding box on the image
        cv2.rectangle(cv_image, (predicted_state[0], predicted_state[1]), (predicted_state[0] + predicted_state[2], predicted_state[1] + predicted_state[3]), (0, 255, 0), 2)

        # Publish the result of image that already has bounding box to target object
        bounded_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        bounded_image_msg.header = msg.header
        self.bounded_image_pub.publish(bounded_image_msg)

        # Initialize tracked object message and store the predicted state value into desire message type
        tracked_object_msg = Detection2D()
        tracked_object_msg.header.stamp = rospy.Time.now()
        tracked_object_msg.bbox.center.x = (predicted_state[0] + predicted_state[2] / 2)
        tracked_object_msg.bbox.center.y = (predicted_state[1] + predicted_state[3] / 2)
        tracked_object_msg.bbox.size_x = predicted_state[2]
        tracked_object_msg.bbox.size_y = predicted_state[3]

        # use to publish the message through corresponding topic
        self.tracked_object_pub.publish(tracked_object_msg)

        # Initialize ground truth message and store the ground truth data into desire message type
        ground_truth_msg = Detection2D()
        ground_truth_msg.header.stamp = rospy.Time.now()
        ground_truth_msg.bbox.center.x = (measurement[0] + measurement[2] / 2)
        ground_truth_msg.bbox.center.y = (measurement[1] + measurement[3] / 2)
        ground_truth_msg.bbox.size_x = measurement[2]
        ground_truth_msg.bbox.size_y = measurement[3]

        # use to publish the message through corresponding topic
        self.ground_truth_pub.publish(ground_truth_msg)

        # Initialize matric number message and store the matric value to desire message type
        matric_number_msg = String()
        matric_number_msg.data = "A0268307Y"

        # use to publish the message through corresponding topic
        self.matric_number_pub.publish(matric_number_msg)

if __name__ == '__main__':
    try:
        object_tracker = SingleObjectTracking()
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass