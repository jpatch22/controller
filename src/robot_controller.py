#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('controller')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time

from matplotlib import pyplot as plt
import numpy as np
from std_msgs.msg import Time
from rosgraph_msgs.msg import Clock

#Our class imports
from lanes_class import Lane_Detection
from driver import Driver
from license_plate import License_Plate
from reader import Reader

class controller():

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        self.vel_pub = rospy.Publisher("/R1/cmd_vel/", Twist, queue_size=1)
        self.license = rospy.Publisher("/license_plate",String,queue_size=10)
        # time = rospy.get_time()
    
        self.twist = Twist()
        self.Lane_Detection = Lane_Detection()
        self.Driver = Driver()
        self.License_Plate = License_Plate()
        self.timer_starter = False
        self.start_time = 0
        self.Reader = Reader()

        rate = rospy.Rate(2)

    def number_red(self, image):
        red_min = np.array([0, 0, 195], np.uint8)
        red_max = np.array([5, 5, 255], np.uint8)

        dst = cv2.inRange(image, red_min, red_max)
        return cv2.countNonZero(dst)

    def number_blue(self, image):
        red_min = np.array([0, 0, 0], np.uint8)
        red_max = np.array([80, 5, 5], np.uint8)

        dst = cv2.inRange(image, red_min, red_max)
        return cv2.countNonZero(dst)

    def callback(self,data):
        if rospy.get_time() - self.start_time >= 60:
            self.license.publish(str('idk,idk,-1,9927'))
        if self.timer_starter == False:
            #time.sleep(7) #To allow tensor flow models to load
            self.license.publish(str('idk,idk,0,AB65'))
            self.timer_starter = True
            self.start_time = rospy.get_time()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        

        if rospy.get_time()- self.start_time < 5:
            line_image, main_intercept, turning_intercept = self.Lane_Detection.process_image(cv_image, 'L')
            forward_velocity, angular_velocity = self.Driver.controller(main_intercept, turning_intercept, 'L')
        else:
            line_image, main_intercept, turning_intercept = self.Lane_Detection.process_image(cv_image, 'R')
            forward_velocity, angular_velocity = self.Driver.controller(main_intercept, turning_intercept, 'R')

        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,500)
        fontScale              = 0.5
        fontColor              = (255,255,255)
        lineType               = 2

        cv2.putText(line_image, "angular_velocity : {}, forward_velocity: {}".format(angular_velocity, forward_velocity), 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)
        
        self.twist.linear.x = forward_velocity
        self.twist.angular.z = angular_velocity
        self.vel_pub.publish(self.twist)

        

        # cv2.imshow("contours",edged)
        x = int(self.number_blue(cv_image))
        if x > 75:
            image2, car_found = self.License_Plate.find_Car(cv_image)
            if(car_found == True):
                cropped = self.License_Plate.get_license_region(image2)
                #print("cropped", cropped)
                if cropped.size == 0:
                    print('HERE') #For some reason removing breaks code lol
                else:
                    crop_height, crop_width, useless = cropped.shape

                    Parking_number_image = cropped[int(1 * crop_height / 5):int(6 * crop_height / 7.0),int(crop_width / 2):int(crop_width * 9.0 / 10)]
                    predicted_p_id = self.Reader.get_parking_id(Parking_number_image)
                    license_plate_image = cropped[2*int(crop_height / 3):, int(crop_width * 1.5 / 10.0): int(crop_width * 8.5 / 10.0)]
                    (lp1, lp2, lp3, lp4) = self.Reader.get_lisence_plates(license_plate_image)

                    print("P_ID: ", predicted_p_id, "Predicted Lisence Plate: ", lp1, lp2, lp3, lp4)

                
        else:
            image2 = cv_image
        image2 = cv2.addWeighted(image2, 0.8, line_image, 1, 1)

        cv2.imshow("Debugging window",image2)
        
        # cv2.imshow("li", line_image)
        # print(image2.shape, line_image.shape, cv_image)
        
        cv2.waitKey(5)


def main(args):
    rospy.init_node('controller', anonymous=True)
    ic = controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)