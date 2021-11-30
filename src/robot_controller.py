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
from pedesterian import Ped_Detection
from vehicle import Vehicle_Detection

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
        self.Ped_Detection = Ped_Detection()
        self.Vehicle_Detection = Vehicle_Detection()

        self.time_crosswalk_1 = 100000

        self.at_crosswalk = False

        self.time_detect_lp_1 = 100000
        self.start_turn_in_2 = 100000
        self.length_of_turn_in = 4
        self.length_of_turn_in_2 = 3 + 1.0
        self.turn_delay = 2

        self.read_penultimate = -100000
        self.penultimate_dt = 2

        self.cardetected = False
        self.turn_in_one = False

        rate = rospy.Rate(2)

    def number_red(self, image):
        red_min = np.array([0, 0, 195], np.uint8)
        red_max = np.array([5, 5, 255], np.uint8)

        dst = cv2.inRange(image, red_min, red_max)
        return cv2.countNonZero(dst)

    def number_blue(self, image):
        red_min = np.array([0, 0, 0], np.uint8)
        red_max = np.array([95, 5, 5], np.uint8)

        dst = cv2.inRange(image, red_min, red_max)
        return cv2.countNonZero(dst)

    def callback(self,data):
        if rospy.get_time() - self.start_time >= 240:
            self.license.publish(str('idk,idk,-1,9927'))
        if self.timer_starter == False:
            time.sleep(15) #To allow tensor flow models to load
            # self.twist.linear.x = 0.1
            # self.twist.angular.z = 0.05
            # self.vel_pub.publish(self.twist)
            # time.sleep(1)
            self.license.publish(str('idk,idk,0,AB65'))
            self.timer_starter = True
            self.start_time = rospy.get_time()

        if rospy.get_time() - self.time_crosswalk_1 > 5:
            self.at_crosswalk = False

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        line_image = np.zeros_like(cv_image)

        # cv2.imshow("Raw", cv_image)

        if self.number_red(cv_image) > 10000 and self.at_crosswalk == False:
            drive = self.Ped_Detection.drive_or_not(cv_image)
            line_image = np.zeros_like(cv_image)
            if drive:
                line_image, main_intercept, turning_intercept = self.Lane_Detection.process_image(cv_image, 'R')
                forward_velocity, angular_velocity = self.Driver.controller(main_intercept, turning_intercept, 'R')
                self.at_crosswalk = True
                self.time_crosswalk_1 = rospy.get_time()
            elif drive == False:
                forward_velocity = 0.0
                angular_velocity = 0.0
        elif rospy.get_time() - self.start_time < 3:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            self.vel_pub.publish(self.twist)
            rospy.sleep(1.8)
            self.twist.linear.x = 0
            self.twist.angular.z = 0.7
            self.vel_pub.publish(self.twist)
            rospy.sleep(2.2)
            forward_velocity = 0
            angular_velocity = 0
        elif self.time_detect_lp_1 < rospy.get_time() < self.time_detect_lp_1 + self.length_of_turn_in:
            print("Turning in")
            line_image, main_intercept, turning_intercept = self.Lane_Detection.process_image(cv_image, 'L')
            forward_velocity, angular_velocity = self.Driver.controller(main_intercept, turning_intercept, 'L')
            self.turn_in_one = True
        elif self.turn_in_one == True and self.cardetected == False and rospy.get_time > self.time_detect_lp_1 + self.length_of_turn_in:
            forward_velocity = 0
            angular_velocity = 0
            if self.Vehicle_Detection.drive_or_not(cv_image) == True:
                self.cardetected = True
                self.start_turn_in_2 = rospy.get_time()
                rospy.sleep(1)
        elif self.cardetected == True and rospy.get_time() < self.start_turn_in_2 + self.length_of_turn_in_2:
            print("left Loop")
            line_image, main_intercept, turning_intercept = self.Lane_Detection.process_image(cv_image, 'L')
            forward_velocity, angular_velocity = self.Driver.controller(main_intercept, turning_intercept, 'L')
        elif rospy.get_time() < self.penultimate_dt + self.read_penultimate:
            forward_velocity = 0.2
            angular_velocity = 0.05
        else:
            line_image, main_intercept, turning_intercept = self.Lane_Detection.process_image(cv_image, 'R')
            forward_velocity, angular_velocity = self.Driver.controller(main_intercept, turning_intercept, 'R')


        print("cardetected", self.cardetected)
        # if rospy.get_time() > self.time_detect_lp_1 + self.length_of_turn_in:
        #     print("finished turning in")

        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,500)
        fontScale              = 0.5
        fontColor              = (255,255,255)
        lineType               = 2

        # cv2.putText(line_image, "angular_velocity : {}, forward_velocity: {}".format(angular_velocity, forward_velocity), 
        #     bottomLeftCornerOfText, 
        #     font, 
        #     fontScale,
        #     fontColor,
        #     lineType)
        
        self.twist.linear.x = forward_velocity
        self.twist.angular.z = angular_velocity
        self.vel_pub.publish(self.twist)


        # cv2.imshow("contours",edged)
        x = int(self.number_blue(cv_image))
        
        if x > 40:
            image2, car_found = self.License_Plate.find_Car(cv_image)
            #print("In reader loop", car_found)
            if(car_found == True):
                #print("Front of car_detected")
                cropped = self.License_Plate.get_license_region(image2)
                #print("cropped", cropped)
                if cropped.size == 0:
                    print('HERE') #For some reason removing breaks code lol
                else:
                    crop_height, crop_width, useless = cropped.shape

                    Parking_number_image = cropped[int(1 * crop_height / 5):int(6 * crop_height / 7.0),int(crop_width / 2):int(crop_width * 9.0 / 10)]
                    (predicted_p_id, c1) = self.Reader.get_parking_id(Parking_number_image)
                    license_plate_image = cropped[int(2*crop_height / 3):, int(crop_width * 1.5 / 10.0): int(crop_width * 8.5 / 10.0)]
                    (lp1, lp2, lp3, lp4, c2) = self.Reader.get_lisence_plates(license_plate_image)
                    if c1 > 0.99 and c2 > 0.99:
                        message = predicted_p_id[0] + ',' + lp1 + lp2 + lp3 + lp4
                        self.license.publish(str("Miti, we love you," + message))
                        print(message)
                        if predicted_p_id[0] == '3':
                            print("Detected last turn in")
                            self.time_detect_lp_1 = rospy.get_time() + self.turn_delay
                        if predicted_p_id[0] == '7':
                            print("Detected CAR")
                            self.read_penultimate = rospy.get_time()

                    # print("P_ID: ", predicted_p_id, "Predicted Lisence Plate: ", lp1, lp2, lp3, lp4, "C1 ", c1, "C2", c2)

                
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