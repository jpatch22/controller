import cv2
import numpy as np
import matplotlib.pyplot as plt

class Line_Follower:
    def __init__(self):

        self.number_slices = 10
        self.slice_size = 100

        self.ROI_reduce_fac = 500

        self.left_correct_intercept = 160
        self.right_correct_intercept = 1180

    def canny(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        u_g = np.array([0, 0, 180])
        l_g = np.array([0, 0, 150])
        mask = cv2.inRange(hsv, l_g, u_g)

        return mask

    def left_display_lines(self, image, lines):
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 30)

        return line_image

    def right_display_lines(self, image, lines):
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 30)

        return line_image

    def display_heading_line(self, image, lines):
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 30)

        return line_image

    def left_region_of_interest(self, image, start_y, end_y, num_slices):
        height, width = image.shape
        polygons = np.array([
        [(0 , start_y), (width // 2 , start_y), (width // 2, end_y), (0, end_y)]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, polygons, 255)
        masked_image = cv2.bitwise_and(image, mask)
        # cv2.imshow('ML', masked_image)
        #cv2.waitKey(0)
        return masked_image

    def right_region_of_interest(self, image, start_y, end_y, num_slices):
        height, width = image.shape
        polygons = np.array([
        [(width // 2 , start_y), (width , start_y), (width, end_y), (width // 2, end_y)]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, polygons, 255)
        masked_image = cv2.bitwise_and(image, mask)
        # cv2.imshow('MR', masked_image)
        # cv2.waitKey(2)
        return masked_image

    def averaged_lines(self, image, lines, start_y, end_y):
        fit = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            fit.append((slope, intercept))
        fit_average = np.average(fit, axis = 0)
        line = self.make_coordinates(image, fit_average, start_y, end_y)

        return np.array([line])

    def make_coordinates(self, image, line_parameters,start_y, end_y):
        slope, intercept = line_parameters
        y1 = start_y
        y2 = end_y
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        return np.array([x1, y1, x2, y2])

    def calc_intercept(self, line, height):
        x1, y1, x2, y2 = line[0].reshape(4)
        slope = (x2 - x1) / (y2 - y1)
        b = x2 - slope * y2
        intercept = slope * height + b
        return int(intercept)

    def process_image(self, image):
        """
        Input: Robot's view in the ROS world
        Output: (combo_image, left_averaged_lines, right_averaged_lines)
            combo_image: Original image with lane lines superimposed
            averaged_lines: numpy array of numpy array that contains [x1 y1 x2 y2]
        """

        lane_image = np.copy(image)
        height, width, channels = lane_image.shape
        canny_image = self.canny(lane_image)

        left_lane_intercepts = []
        right_lane_intercepts = []

        line_image = np.zeros(lane_image.shape, dtype=np.uint8)
        for i in range(0, self.number_slices):
            start_y = height - i * self.slice_size
            end_y = height - (i + 1) * self.slice_size
            left_cropped_image = self.left_region_of_interest(canny_image, start_y, end_y, i)
            right_cropped_image = self.right_region_of_interest(canny_image, start_y, end_y, i)

            left_lines = cv2.HoughLinesP(left_cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=2, maxLineGap=50)
            right_lines = cv2.HoughLinesP(right_cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=2, maxLineGap=50)


            if left_lines is not None:
                left_averaged_lines = self.averaged_lines(lane_image, left_lines, start_y, end_y)
                left_line_image = self.left_display_lines(lane_image, left_averaged_lines)
                if i == 0:
                    left_lane_bottom = left_averaged_lines[0][0]
                else:
                    left_lane_bottom = self.calc_intercept(left_averaged_lines, height)
            else:
                left_averaged_lines = None
                left_line_image = None
                if i == 0:
                    left_lane_bottom = 0
                else:
                    left_lane_bottom = self.left_correct_intercept

            if right_lines is not None:
                right_averaged_lines = self.averaged_lines(lane_image, right_lines, start_y, end_y)
                right_line_image = self.right_display_lines(lane_image, right_averaged_lines)
                if i == 0:
                    right_lane_bottom = right_averaged_lines[0][0]
                else:
                    right_lane_bottom = self.calc_intercept(right_averaged_lines, height)
            else:
                right_averaged_lines = None
                right_line_image = None
                if i == 0:
                    right_lane_bottom = width
                else:
                    right_lane_bottom = self.right_correct_intercept

            left_lane_intercepts.append(left_lane_bottom)
            right_lane_intercepts.append(right_lane_bottom)

            if left_line_image is not None:
                line_image += left_line_image
                line_image = cv2.circle(line_image, (left_lane_bottom, height - 50), 50, (255, 0, 0), -1)
            if right_line_image is not None:
                line_image += right_line_image
                line_image = cv2.circle(line_image, (right_lane_bottom, height - 50), 50, (0, 255, 0), -1)

        # combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)

        return (line_image, left_lane_intercepts, right_lane_intercepts)
