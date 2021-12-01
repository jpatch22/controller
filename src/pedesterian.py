import cv2
import numpy as np

class Ped_Detection:
	def __init__(self):
		pass

	def hsv_filter(self, image):
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, np.array([91,54,49]) , np.array([113,138,110]))
		return mask

	def get_pedesterian_location(self, mask):
		im, contours, hier = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		contours = sorted(contours, key=cv2.contourArea, reverse=True)
		color_version = cv2.merge((mask, mask, mask))
		(x_min, y_min, width, height) = cv2.boundingRect(contours[0])
		cv2.circle(color_version, (x_min, y_min), 25, (0, 255, 0), -1)
		#cv2.imshow("mask", color_version)

		if width * height > 300:
			ped_detected = True
		else:
			ped_detected = False

		return (x_min, y_min, ped_detected)

	def drive_or_not(self, image):
		filter_a = self.hsv_filter(image)
		(x, y, ped_detected) = self.get_pedesterian_location(filter_a)
		if x > 580 and x < 750 and ped_detected == True:
			return True
		else:
			return False