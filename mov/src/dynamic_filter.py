#!/usr/bin/env python
from __future__ import print_function
import math
# from  nav_msgs.msg import Odometry
from  geometry_msgs.msg import Twist
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from cv2 import line
import numpy as np
class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

    def hsv_filter(self,img):
        def nothing(x):
            pass

        # Create a window
        cv.namedWindow('image')

        # create trackbars for color change
        cv.createTrackbar('HMin','image',0,179,nothing) # Hue is from 0-179 for Opencv
        cv.createTrackbar('SMin','image',0,255,nothing)
        cv.createTrackbar('VMin','image',0,255,nothing)
        cv.createTrackbar('HMax','image',0,179,nothing)
        cv.createTrackbar('SMax','image',0,255,nothing)
        cv.createTrackbar('VMax','image',0,255,nothing)

        # Set default value for MAX HSV trackbars.
        cv.setTrackbarPos('HMax', 'image', 179)
        cv.setTrackbarPos('SMax', 'image', 255)
        cv.setTrackbarPos('VMax', 'image', 255)

        # Initialize to check if HSV min/max value changes
        hMin = sMin = vMin = hMax = sMax = vMax = 0
        phMin = psMin = pvMin = phMax = psMax = pvMax = 0

        # img = cv.imread('/home/ros/hsv_filter/unknown.png')
        output = img
        # waitTime = 33

        while(1):
            # get current positions of all trackbars
            hMin = cv.getTrackbarPos('HMin','image')
            sMin = cv.getTrackbarPos('SMin','image')
            vMin = cv.getTrackbarPos('VMin','image')

            hMax = cv.getTrackbarPos('HMax','image')
            sMax = cv.getTrackbarPos('SMax','image')
            vMax = cv.getTrackbarPos('VMax','image')

            # Set minimum and max HSV values to display
            lower = np.array([hMin, sMin, vMin])
            upper = np.array([hMax, sMax, vMax])

            # Create HSV Image and threshold into a range.
            hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
            mask = cv.inRange(hsv, lower, upper)
            output = cv.bitwise_and(img,img, mask= mask)

            # Print if there is a change in HSV value
            if( (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
                print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
                phMin = hMin
                psMin = sMin
                pvMin = vMin
                phMax = hMax
                psMax = sMax
                pvMax = vMax

            # Display output image
            return output    
        def callback(self,data):

            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
            src = cv_image
            ROI = src[77:581,130:480]
            filter = self.hsv_filter(ROI)
            dst = cv.Canny(filter, 50, 200, None, 3)
            # Copy edges to the images that will display the results in BGR
            cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
            lines = cv.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
            # print(lines)
            vel = Twist()
            S = 0
            if lines is not None:
                for i in range(0, len(lines)):
                    rho = lines[i][0][0]
                    theta = lines[i][0][1]
                    a = math.cos(theta)
                    b = math.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                    theta_approx  = round(theta,2)
                    for i in range(0,10):
                        if i == 0:
                            S = 0
                        S+=theta_approx
                    S= S/10
                    if S>1:
                        vel.angular.z = 1
                        vel.linear.x  = 0
                        self.cmd_pub.publish(vel)
                        print("turn",S,rho)
                    else:
                        vel.linear.x = 0.5
                        vel.angular.z = 0
                        self.cmd_pub.publish(vel)
                        print("st",S,rho)
                    cv.line(cdst, pt1, pt2, (0,0,255), 3, cv.LINE_AA)
            cv.imshow("Image window", cdst)
            cv.imshow("IMG filter",filter)
            cv.waitKey(3)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cdst, "bgr8"))
            except CvBridgeError as e:
            #   print(e)
                pass


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)