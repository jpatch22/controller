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

from matplotlib import pyplot as plt
import numpy as np

#Our class imports
from lanes_class import Lane_Detection
from lane_controller import Lane_Controller
from driver import Driver

class controller():

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        self.vel_pub = rospy.Publisher("/R1/cmd_vel/", Twist, queue_size=1)
        rate = rospy.Rate(2)
        self.twist = Twist()
        self.Lane_Detection = Lane_Detection()
        self.Driver = Driver()

    def callback(self,data):
        try:
              cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
              print(e)

        combo_image, hl = self.Lane_Detection.process_image(cv_image)
        forward_velocity, angular_velocity = self.Driver.controller(hl, cv_image.shape[1])

        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,500)
        fontScale              = 0.5
        fontColor              = (255,255,255)
        lineType               = 2

        cv2.putText(combo_image, "angular_velocity : {}, forward_velocity: {}".format(angular_velocity, forward_velocity), 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)

        self.twist.linear.x = forward_velocity
        self.twist.angular.z = angular_velocity
        self.vel_pub.publish(self.twist)

        cv2.imshow("Image window", combo_image)
        cv2.imshow("Canny", self.Lane_Detection.canny(cv_image))
        cv2.waitKey(3)


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