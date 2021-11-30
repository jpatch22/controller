import cv2
import numpy as np

class Vehicle_Detection:
	def __init__(self):
		pass

	def hsv_filter(self, image):
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, np.array([0,0,0]) , np.array([0,0,38]))
		return mask

	def get_vehicle_size(self, mask):
		im, contours, hier = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		contours = sorted(contours, key=cv2.contourArea, reverse=True)
		color_version = cv2.merge((mask, mask, mask))
		(x_min, y_min, width, height) = cv2.boundingRect(contours[0])
		cv2.rectangle(color_version, (x_min, y_min), (x_min + width, y_min + height), (0,255,0), 4)
		#cv2.imshow("color", color_version)
		return width * height

	def drive_or_not(self, image):
		height, width, channels = image.shape
		# image = image[height // 2:,:]
		filter_a = self.hsv_filter(image)
		area = self.get_vehicle_size(filter_a)
		print(area)
		if area > 22000:
			return True
		else:
			return False