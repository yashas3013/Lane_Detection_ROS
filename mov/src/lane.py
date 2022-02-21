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
  def turn(self,time,dir):
    pass
    
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    src = cv_image
    ROI = src[77:581,130:480]
    # ROI = src[175:450,200:450]
    # ROI = src
    hsv = cv.cvtColor(ROI, cv.COLOR_BGR2HSV)
# Threshold of blue in HSV space
    lower_red = np.array([40,255,33])
    upper_red = np.array([179,255,255])
# preparing the mask to overlay
    mask = cv.inRange(hsv, lower_red, upper_red)
    red = cv.bitwise_and(ROI, ROI, mask = mask)
    dst = cv.Canny(ROI, 50, 200, None, 3)
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
    cv.imshow("fiter",red)
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