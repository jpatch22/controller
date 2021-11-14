import cv2
import numpy as np
import matplotlib.pyplot as plt

class Lane_Detection:
    def __init__(self):
        self.cutoff = 150
        self.shift_ratio = 1.0 / 2.2

    def canny(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        canny = cv2.Canny(blur, 50, 140)

        return canny

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

    def left_region_of_interest(self, image):
        height, width = image.shape
        polygons = np.array([
        [(0, height), (width // 2, height), (width // 2, height - self.cutoff), (0, height - self.cutoff)]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, polygons, 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def right_region_of_interest(self, image):
        height, width = image.shape
        polygons = np.array([
        [(width // 2, height), (width, height), (width, height - self.cutoff), (width // 2, height - self.cutoff)]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, polygons, 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def averaged_lines(self, image, lines):
        fit = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            fit.append((slope, intercept))
        fit_average = np.average(fit, axis = 0)
        line = self.make_coordinates(image, fit_average)

        return np.array([line])

    def make_coordinates(self, image, line_parameters):
        slope, intercept = line_parameters
        y1 = image.shape[0]
        y2 = int(y1 * ((y1 - self.cutoff) / y1))
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        return np.array([x1, y1, x2, y2])

    def heading_line_no_none(self, left_averaged_lines, right_averaged_lines):
        x1 = int((left_averaged_lines[0][0] + right_averaged_lines[0][0]) / 2)
        y1 = int((left_averaged_lines[0][1] + right_averaged_lines[0][1]) / 2)
        x2 = int((left_averaged_lines[0][2] + right_averaged_lines[0][2]) / 2)
        y2 = int((left_averaged_lines[0][3] + right_averaged_lines[0][3]) / 2)
        n = np.array([x1, y1, x2, y2])

        return np.array([n])

    def hl_left_none(self, image, right_averaged_lines):
        hl = np.copy(right_averaged_lines)
        shift = self.shift_ratio * image.shape[1]
        print("SHIFT", shift, self.shift_ratio)
        hl[0][0] = right_averaged_lines[0][0] - int(shift)
        hl[0][2] = right_averaged_lines[0][2] - int(shift)
        return hl

    def hl_right_none(self, image, left_averaged_lines):
        hl = np.copy(left_averaged_lines)
        shift = self.shift_ratio * image.shape[1]
        print("SHIFT", shift, self.shift_ratio)
        hl[0][0] = left_averaged_lines[0][0] + int(shift)
        hl[0][2] = left_averaged_lines[0][2] + int(shift)
        return hl

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
        left_cropped_image = self.left_region_of_interest(canny_image)
        right_cropped_image = self.right_region_of_interest(canny_image)
        left_lines = cv2.HoughLinesP(left_cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
        right_lines = cv2.HoughLinesP(right_cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)

        if left_lines is not None:
            left_averaged_lines = self.averaged_lines(lane_image, left_lines)
            left_line_image = self.left_display_lines(lane_image, left_averaged_lines)
            left_lane_bottom = left_averaged_lines[0][0]
        else:
            left_averaged_lines = None
            left_line_image = None
            left_lane_bottom = None
            print("No left line detected")

        if right_lines is not None:
            right_averaged_lines = self.averaged_lines(lane_image, right_lines)
            right_line_image = self.right_display_lines(lane_image, right_averaged_lines)
            right_lane_bottom = right_averaged_lines[0][0]
        else:
            right_averaged_lines = None
            right_line_image = None
            right_lane_bottom = None
            print("No right line detected")

        if left_lines is not None and right_lines is not None:
            hl = self.heading_line_no_none(left_averaged_lines, right_averaged_lines)
        elif left_lines is None and right_lines is None:
            mid = int(width / 2)
            thing = np.array([mid, height, mid, height-self.cutoff])
            hl = np.array([thing])
        elif left_lines is None:
            hl = self.hl_left_none(lane_image, right_averaged_lines)
        elif right_lines is None:
            hl = self.hl_right_none(lane_image, left_averaged_lines)



        heading_line_image = self.display_heading_line(lane_image, hl)
        line_image = heading_line_image
        if left_line_image is not None:
            line_image += left_line_image
            lane_image = cv2.circle(lane_image, (left_lane_bottom, height - 50), 50, (255, 0, 0), -1)
        if right_line_image is not None:
            line_image += right_line_image
            lane_image = cv2.circle(lane_image, (right_lane_bottom, height - 50), 50, (0, 255, 0), -1)

        combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)

        return (combo_image, hl)
