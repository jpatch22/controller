#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt
# import imutils

class License_Plate():

	def __init__(self):
		self.cardetected = False
		self.license_in_frame = False
		self.ymin = 0
		self.ymax = 100
		self.xmin = 0
		self.xmax = 100

	def find_Car(self,image):
		self.cardetected = False
		canny = cv2.Canny(image, 50, 120)
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, np.array([0,0,95]) , np.array([0,0,220]))
		im,contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		contours = sorted(contours, key=cv2.contourArea, reverse=True)
		if contours:
			(x_min, y_min, box_width, box_height) = cv2.boundingRect(contours[0])
			#print("values", (x_min, y_min, box_width, box_height))
			cv2.rectangle(image, (x_min - 15, y_min -15), (x_min + box_width + 15, y_min + box_height + 50),(0,255,0), 4)
			self.ymin = y_min-15
			self.ymax = y_min+box_height + 50
			self.xmin = x_min -15
			self.xmax = x_min + box_width + 15
			self.cardetected = self.valid_box(box_width, box_height)

			return (image,self.cardetected)
		return (image, self.cardetected)

	def valid_box(self, width, height):
		if width < 60 or width > 150:
			return False
		if height < 50 or height > 200:
			return False
		return True


	def get_license_region(self, image):
		if (self.cardetected == True):
			imagecopy = image[self.ymin:self.ymax,self.xmin:self.xmax]
			return imagecopy
		return image

	








		

