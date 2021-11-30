import numpy as np
import cv2
import tensorflow as tf
from tensorflow.keras import models
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model

sess1 = tf.Session()    
graph1 = tf.get_default_graph()
set_session(sess1)



class Reader:
	def __init__(self):
		self.model_p_id = models.load_model('/home/fizzer/train_cnn/p_id_training/p_id_model')
		self.model_lp_num = models.load_model('/home/fizzer/train_cnn/lp_num_train_data/lp_num_model')
		self.model_lp_letters = models.load_model('/home/fizzer/train_cnn/lp_letter_train_data/lp_letter_model')
		self.p_id_label = 0
		self.lp_label = 0
		pass

	def check_p_id_box_valid(self, box):
		min_width = 100
		max_width = 150
		min_height = 35
		max_height = 55
		for b in list_of_boxes:
			(x_min, y_min, box_width, box_height) = b
			if box_width > max_width or box_width < min_width:
				return False
			if box_height > max_height or box_height < min_height:
				return False 

		return True
		

	def get_parking_id(self, p_id_image):
		ret,thresh = cv2.threshold(p_id_image,50,255,cv2.THRESH_BINARY_INV)
		gray = cv2.cvtColor(thresh, cv2.COLOR_RGB2GRAY)

		_, contours, hier = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		
		sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)[:-1]

		
		#cv2.drawContours(thresh, sorted_contours[0], -1, (0,0,255), 3)
		# cv2.imshow('gray', gray)
		# cv2.imshow('thresh', thresh)

		if sorted_contours is not None and len(sorted_contours) != 0:
			(x_min, y_min, box_width, box_height) = cv2.boundingRect(sorted_contours[0])
			cv2.rectangle(thresh, (x_min - 10, y_min - 10), (x_min + box_width + 10, y_min + box_height + 10),(0,255,0), 4)
			

			resized_image = cv2.resize(thresh[y_min:y_min + box_height, x_min:x_min+box_width], (140, 140))
			
			#cv2.imshow('resize', resized_image) #p_id debug
			cv2.imwrite("/home/fizzer/train_cnn/p_id_auto_collect_data/{}.png".format(self.p_id_label), resized_image)
			self.p_id_label += 1
			
			p_id_cnn = np.array([resized_image])
			global sess1
			global graph1

			with graph1.as_default():
				set_session(sess1)
				prediction = self.model_p_id.predict(p_id_cnn)
				predicted_num = self.convert_nums(prediction)
				return (predicted_num, np.amax(prediction[0]))
		return (0,0)
		

	def convert_nums(self, array):
		conv = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"]
		labels = []
		for row in array:
			index = np.argmax(row)
			labels.append(conv[index])
		return labels

	def convert_letters(self, array):
		conv = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"]
		labels = []
		for row in array:
			index = np.argmax(row)
			labels.append(conv[index])
		return labels

	def get_lisence_plates(self, licence_image):
		resized_image = cv2.resize(licence_image, (800, 200))
		#cv2.imshow("input", resized_image)
		
		hsv = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
		
		mask2 = cv2.inRange(hsv, np.array([65,65,65]) , np.array([255,255,255]))
		# cv2.imshow("pre contour", mask2)
		
		ima_msk = cv2.merge((mask2, mask2, mask2))
		img_copy = ima_msk.copy()

		
		# mask_gray = cv2.cvtColor(mask2, cv2.COLOR_BGR2GRAY)

		_, contours, hier = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)[:4]
		left_to_right_contours = sorted(sorted_contours, key=self.get_cX)

		box_list = []

		for contour in left_to_right_contours:
			cv2.drawContours(ima_msk, [contour], -1, (0,0,255), 3)
			(x_min, y_min, box_width, box_height) = cv2.boundingRect(contour)
			box_list.append((x_min, y_min, box_width, box_height))

			cv2.rectangle(ima_msk, (x_min - 10, y_min - 10), (x_min + box_width + 10, y_min + box_height + 10),(0,255,0), 4)

		split_boxes = list(box_list)
		for i in range(0, len(box_list)):
			if box_list[i][2] > 200:
				(x1, y1, w1, h1, x2, y2, w2, h2) = self.split(box_list[i][0], box_list[i][1], box_list[i][2], box_list[i][3])
				split_boxes.remove(box_list[i])
				split_boxes.insert(i, (x1, y1, w1, h1))
				split_boxes.insert(i+1, (x2, y2, w2, h2))
			elif box_list[i][3] < 35:
				split_boxes.remove(box_list[i])

		split_boxes = sorted(split_boxes, key=self.rectangle_area, reverse=True)
		split_boxes = sorted(split_boxes, key=self.l_to_r_box)
		box_list = split_boxes[:4]

		for sb in box_list:
			(x_min, y_min, box_width, box_height) = sb
			cv2.rectangle(img_copy, (x_min - 10, y_min - 10), (x_min + box_width + 10, y_min + box_height + 10),(0,255,0), 4)

		#cv2.imshow('copy', img_copy) #lisence plate debugger

		lp_letters = []
		lp_nums = []
		confidence = []
		# print(self.check_box_validity(box_list))
		if self.check_box_validity(box_list):
			i = 0
			for b in box_list:
				# print(b)
				lp = cv2.resize(mask2[b[1]:b[1] + b[3], b[0]:b[0] + b[2]], (140, 140))
				if i < 2:
					lp_letters.append(cv2.merge((lp, lp, lp)))
				else: 
					lp_nums.append(cv2.merge((lp, lp, lp)))
				i += 1
			# print("SHAPE", lp_nums[0].shape)
			if len(lp_nums) !=0 and len(lp_letters) != 0:
				# cv2.imwrite("/home/fizzer/train_cnn/lp_auto_collect_data/{}.png".format(self.lp_label), lp_letters[0])
				# cv2.imwrite("/home/fizzer/train_cnn/lp_auto_collect_data/{}.png".format(self.lp_label + 1), lp_letters[1])
				# cv2.imwrite("/home/fizzer/train_cnn/lp_auto_collect_data/{}.png".format(self.lp_label + 2), lp_nums[0])
				# cv2.imwrite("/home/fizzer/train_cnn/lp_auto_collect_data/{}.png".format(self.lp_label + 3), lp_nums[1])
				if lp_nums[0].shape == (140, 140, 3) and lp_nums[1].shape == (140, 140, 3):
					self.lp_label += 4

					lp_letters_na = np.asarray(lp_letters)
					lp_nums_na = np.asarray(lp_nums)

					global sess1
					global graph1
					letter_p = [] 
					num_p = []
					with graph1.as_default():
						set_session(sess1)
						letter_prediction_arrays = self.model_lp_letters.predict(lp_letters_na)
						letter_p = self.convert_letters(letter_prediction_arrays)
						num_prediction_arrays = self.model_lp_num.predict(lp_nums_na)
						num_p = self.convert_nums(num_prediction_arrays)

						confidence.append(np.amax(letter_prediction_arrays[0]))
						confidence.append(np.amax(letter_prediction_arrays[1]))
						confidence.append(np.amax(num_prediction_arrays[0]))
						confidence.append(np.amax(num_prediction_arrays[1]))
					return (letter_p[0], letter_p[1], num_p[0], num_p[1], min(confidence))
				
		# cv2.imshow("mask2", ima_msk)
		return (0, 0, 0, 0, 0)

		

	def split(self, xmin, y_min, box_width, box_height):
		x1 = xmin
		y1 = y_min
		y2 = y_min
		w1 = box_width // 2
		w2 = box_width // 2
		x2 = x1 + w1
		h1 = box_height
		h2 = box_height
		return (x1, y1, w1, h1, x2, y2, w2, h2)

	def check_box_validity(self, list_of_boxes):
		min_width = 50
		max_width = 200
		min_height = 25
		max_height = 300
		for b in list_of_boxes:
			(x_min, y_min, box_width, box_height) = b
			if box_width > max_width or box_width < min_width:
				return False
			if box_height > max_height or box_height < min_height:
				return False 

		return True

	def get_cX(self, s):
		M = cv2.moments(s)
		cX = int(M["m10"] / M["m00"])
		return cX

	def l_to_r_box(self, box):
		(x1, y1, width, height) = box
		return x1

	def rectangle_area(self, box):
		(x1, y1, width, height) = box
		return width * height