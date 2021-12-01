import numpy as np
import cv2

class Middle:
	def __init__(self):
		self.right_forward_speed_reduction_constant = 10 ** (-2) * 3

	def middle_drive(self, image):
		gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
		h, w = gray.shape
		gray_crop = gray[1*h/2:h,w/8:7*w/8]
		h2 , w2 = gray_crop.shape
		ret, mask = cv2.threshold(gray_crop, thresh = 92,maxval = 255, type = cv2.THRESH_BINARY_INV)
		middle = w2/2
		M = cv2.moments(mask)
		cx = middle + 300
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
		error = cx - middle
		angular_velocty = -float(error)/35 - 0.12
		forward_velocity = 0.45
		# cv2.circle(mask, (cx, h2/2), 20, (255, 0, 0), -1)
		# cv2.imshow("mask", mask)

		if forward_velocity - self.right_forward_speed_reduction_constant * angular_velocty < 0:
			forward_velocity = 0
		else:
			forward_velocity = forward_velocity - self.right_forward_speed_reduction_constant * abs(angular_velocty)

		return (forward_velocity, angular_velocty)
