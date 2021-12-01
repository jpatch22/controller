import cv2
import numpy as np
import matplotlib.pyplot as plt

class Lane_Detection:
    def __init__(self):
        self.cutoff = 155
        self.right_correct_intercept = 1050

    def hsv_filter(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # cv2.imshow("hsv", hsv)
        mask = cv2.inRange(hsv, np.array([0,0,70]) , np.array([255,160,255]))
        return mask

    def canny(self, image):
        blur = cv2.GaussianBlur(image, (5,5), 0)
        canny = cv2.Canny(blur, 50, 140)

        return canny

    def averaged_lines(self, line_starting, line_vert, lines):
        fit = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            fit.append((slope, intercept))
        fit_average = np.average(fit, axis = 0)
        line = self.make_coordinates(line_starting, line_vert, fit_average)

        return np.array([line])

    def make_coordinates(self, y1, height, line_parameters):
        slope, intercept = line_parameters
        y2 = int(y1 * ((float(y1) - height) / y1))
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        return np.array([x1, y1, x2, y2])

    def right_region_of_interest(self, image):
        height, width = image.shape
        polygons = np.array([
        [(width // 2, height), (width, height), (width, height - self.cutoff), (width // 2, height - self.cutoff)]
        ])
        mask = np.zeros_like(image)
        x = cv2.fillPoly(mask, polygons, 255)
        # cv2.imshow("x", x)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def right_turning_region(self, image):
        height, width = image.shape
        polygons = np.array([
        [(620, 470), (820, 470), (820, 410), (620,410)]
        ])
        mask = np.zeros_like(image)
        x = cv2.fillPoly(mask, polygons, 255)
        # cv2.imshow("x", x)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def right_display_lines(self, image, lines):
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                try:
                    cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 30)
                except:
                    pass

        return line_image

    def left_display_lines(self, image, lines):
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                try:
                    cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 30)
                except:
                    pass

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

    def calc_intercept(self, line, height):
        x1, y1, x2, y2 = line[0].reshape(4)
        slope = (x2 - x1) / (y2 - y1)
        b = x2 - slope * y2
        intercept = slope * height + b
        return int(intercept)

    def follow_right_line(self, img):
        hsv = self.hsv_filter(img)
        blur = cv2.GaussianBlur(hsv, (5,5), 10)
        can = self.canny(blur)
        # cv2.imshow("can", can)
        height, width, channels = img.shape

        right_cropped_image = self.right_region_of_interest(can)
        right_lines = cv2.HoughLinesP(right_cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=20, maxLineGap=10)
        if right_lines is None:
            return (np.zeros(img.shape, dtype=np.uint8), 1400, None)
        right_averaged_lines = self.averaged_lines(height, self.cutoff, right_lines)
        right_line_image = self.right_display_lines(img, right_averaged_lines)
        main_intercept = self.calc_intercept(right_averaged_lines, height)

        right_turning_region = self.right_turning_region(can)
        right_turn_lines = cv2.HoughLinesP(right_turning_region, 2, np.pi/180, 100, np.array([]), minLineLength=20, maxLineGap=10)
        if right_turn_lines is None:
            turning_intercept = self.right_correct_intercept
        else: 
            if right_turn_lines[0][0][1] - right_turn_lines[0][0][3] == 0:
                turning_intercept = 1100 #Subject to change
            else:
                average_turn_lines = self.averaged_lines(470, 60, right_turn_lines)
                right_line_image += self.right_display_lines(img, average_turn_lines)   
                turning_intercept = self.calc_intercept(average_turn_lines, height)
        

        return (right_line_image, main_intercept, turning_intercept)

    def follow_left_line(self, img):
        hsv = self.hsv_filter(img)
        blur = cv2.GaussianBlur(hsv, (5,5), 10)
        can = self.canny(blur)
        #cv2.imshow("can", can)
        height, width, channels = img.shape

        left_cropped_image = self.left_region_of_interest(can)
        left_lines = cv2.HoughLinesP(left_cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=20, maxLineGap=10)
        if left_lines is not None:
            left_averaged_lines = self.averaged_lines(height, self.cutoff, left_lines)
            left_line_image = self.left_display_lines(img, left_averaged_lines)
            main_intercept = self.calc_intercept(left_averaged_lines, height)
        else: 
            left_line_image = np.zeros(img.shape, dtype=np.uint8)
            main_intercept = 0 #Subject

        return (left_line_image, main_intercept, None)

    def process_image(self, img, line_to_follow):
        if line_to_follow == 'R':
            return self.follow_right_line(img)
        else:
            return self.follow_left_line(img)
