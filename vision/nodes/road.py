#!/usr/bin/env python

import sys

import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Road:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('opencv/road',Image,queue_size=10)
        self.image_sub = rospy.Subscriber("/sensor/camera/image_raw",Image,self.callback)
    
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Detect roads
        canny_image = self.canny_edge_detector(cv_image)
        cropped_image = self.region_of_interest(canny_image)
        lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength = 20, maxLineGap = 10)
        if lines is None:
            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_image))
        else:
            averaged_lines = self.average_slope_intercept(cv_image, lines)
            line_image = self.display_lines(cv_image, averaged_lines)
            combo_image = cv2.addWeighted(cv_image, 0.8, line_image, 1, 1)
            self.pub.publish(self.bridge.cv2_to_imgmsg(combo_image))
    
    def canny_edge_detector(self, image):
        # Convert the image color to grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
        
        # Reduce noise from the image
        blur = cv2.GaussianBlur(gray_image, (5, 5), 0) 
        canny = cv2.Canny(blur, 50, 150)
        return canny

    def region_of_interest(self, image):
        height, width = image.shape[:2]
        polygons = np.array([[(0, height), (0, 150), (width, 150), (width, height)]])
        mask = np.zeros_like(image)
        
        # Fill poly-function deals with multiple polygon
        cv2.fillPoly(mask, polygons, 255) 
        
        # Bitwise operation between canny image and mask image
        masked_image = cv2.bitwise_and(image, mask) 
        return masked_image

    def create_coordinates(self, image, line_parameters):
        try:
            slope, intercept = line_parameters
        except TypeError:
            slope, intercept = 0.001, 0
        y1 = image.shape[0]
        y2 = int(y1 * (3 / 5))
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return np.array([x1, y1, x2, y2])

    def average_slope_intercept(self, image, lines):
        left_fit = []
        right_fit = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            # It will fit the polynomial and the intercept and slope
            parameters = np.polyfit((x1, x2), (y1, y2), 1) 
            slope = parameters[0]
            intercept = parameters[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
                
        left_fit_average = np.average(left_fit, axis = 0)
        right_fit_average = np.average(right_fit, axis = 0)
        left_line = self.create_coordinates(image, left_fit_average)
        right_line = self.create_coordinates(image, right_fit_average)
        return np.array([left_line, right_line])
    
    def display_lines(self, image, lines):
        line_image = np.zeros_like(image)
        if lines is not None:
            for x1, y1, x2, y2 in lines:
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
        return line_image