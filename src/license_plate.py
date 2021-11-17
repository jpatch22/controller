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

	def read_number(self,image):
		hgt, wdt,_ = image.shape
		canny = cv2.Canny(image, 50, 120)
		rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		# Grayscale
		gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		# Canny edges
		edged = cv2.Canny(gray, 30, 200)
		# Finding Contours
		contour, hier = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		# All contours
		cv2.drawContours(rgb, contours, -1, (0,255,0), 3)
	
		return rgb

	def find_Car(self,image):
		self.cardetected = False
		canny = cv2.Canny(image, 50, 120)
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		# blue values for cars below
		# mask = cv2.inRange(hsv, np.array([120,40,2]) , np.array([126,255,255]))
		mask = cv2.inRange(hsv, np.array([0,0,98]) , np.array([0,0,220]))
		im,contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		contours = sorted(contours, key=cv2.contourArea, reverse=True)
		if contours:
			self.cardetected = True
			(x_min, y_min, box_width, box_height) = cv2.boundingRect(contours[0])
			cv2.rectangle(image, (x_min - 15, y_min -15), (x_min + box_width + 15, y_min + box_height + 50),(0,255,0), 4)
			self.ymin = y_min-15
			self.ymax = y_min+box_height + 50
			self.xmin = x_min -15
			self.xmax = x_min + box_width + 15
			return (image,self.cardetected)
		return (image, self.cardetected)

	def get_license_region(self, image):
		if (self.cardetected == True):
			imagecopy = image[self.ymin:self.ymax,self.xmin:self.xmax]
			return imagecopy
		return image

	








		

