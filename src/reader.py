import numpy as np
import cv2
from tensorflow import keras

class Reader:
	def __init__(self):
		#self.model = keras.models.load_model('~/cnn_trainer/model_dir/my_model')
		# print(model)
		pass

	def get_parking_id(self, p_id_image):
		ret,thresh = cv2.threshold(p_id_image,50,255,cv2.THRESH_BINARY_INV)
		resized_image = cv2.resize(thresh, (140, 140))
		cv2.imshow("p_id_image", thresh)
		return

	def get_lisence_plates(self, licence_image):
		resized_image = cv2.resize(licence_image, (800, 200))

		# cv2.imshow("gray", thresh)
		

		hsv = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
		# mask1 = cv2.inRange(hsv, np.array([0,0,98]) , np.array([0,0,220])) Possibly useful for warp perspective!
		mask2 = cv2.inRange(hsv, np.array([80,80,80]) , np.array([255,255,255]))
		lp_1 = mask2[80:150, 50:190]
		lp_2 = mask2[80:150, 190:330]
		lp_3 = mask2[80:150, 470:600]
		lp_4 = mask2[80:150, 600:740]
		cv2.imshow("lp_1", lp_1)
		cv2.imshow("lp_2", lp_2)
		cv2.imshow("lp_3", lp_3)
		cv2.imshow("lp_4", lp_4)
		cv2.imshow("mask", mask2)

		return