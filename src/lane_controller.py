import cv2
import numpy as np
import matplotlib.pyplot as plt

class Lane_Controller:
    def __init__(self):
        self.constant_for_angular_from_intercepts = 0.001
        self.constant_for_angular_from_slope = 1
        self.basic_forward_velocity = 0.3

        self.forward_speed_reduction_constant = 0.01

        self.right_correct_intercept = 1000
        self.left_correct_intercept = 200

        self.right_correct_slope = 0.6

    def pid(self, lane_to_follow, left_averaged_lines, right_averaged_lines):
        """
        Input: lane_to_follow -> either left or right
        Input:Two lines of format [[x1 y1 x2 y2]]
        Output: (forward_velocity, angular_velocity)
        """
        forward_velocity = self.basic_forward_velocity
        angular_velocity = 0
        calculated_right_slope = 0
        calculated_left_slope = 0

        if lane_to_follow == 'L':
            if left_averaged_lines is None:
                return (self.basic_forward_velocity, 0, 0, 0)
            calculated_left_slope = (left_averaged_lines[0][3] - left_averaged_lines[0][1]) / (left_averaged_lines[0][2] - left_averaged_lines[0][0])
            calculated_left_intercept = left_averaged_lines[0][0]
            angular_velocity += self.constant_for_angular_from_intercepts * (self.left_correct_intercept - calculated_left_intercept)
        else:
            if right_averaged_lines is None:
                return (self.basic_forward_velocity, 0, 0, 0)
            #Slope portion
            calculated_right_slope = float((right_averaged_lines[0][3] - right_averaged_lines[0][1])) / (right_averaged_lines[0][2] - right_averaged_lines[0][0])
            angular_velocity += self.constant_for_angular_from_slope * (self.right_correct_slope - calculated_right_slope)

            print('RIGHT', right_averaged_lines, calculated_right_slope)
            calculated_right_intercept = right_averaged_lines[0][0]
            angular_velocity += self.constant_for_angular_from_intercepts * (self.right_correct_intercept - calculated_right_intercept)

        forward_velocity -= self.forward_speed_reduction_constant * np.abs(angular_velocity)

        return (forward_velocity, angular_velocity, calculated_left_slope, calculated_right_slope)

    def old_pid(self, cv_image):
        (height,width,channels) = cv_image.shape
        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        ret, mask = cv2.threshold(gray, thresh=140, maxval=255, type=cv2.THRESH_BINARY_INV)
        M = cv2.moments(mask)
        if M['m00'] > 0:
            center_x = int(M["m10"]/M["m00"])
            center_y = int(M["m01"]/M["m00"])

        steering_value = center_x - width / 2

        return (self.basic_forward_velocity, -float(steering_value) / 100, 0, 0)
