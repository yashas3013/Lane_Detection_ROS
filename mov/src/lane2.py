#!/usr/bin/env python
from __future__ import print_function
from dis import dis
import math

from  geometry_msgs.msg import Twist
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

from tf.transformations import euler_from_quaternion
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1)
    self.cmd_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
    self.bridge = CvBridge()
    self.vel = Twist()
    self.vel.linear.x = 1
    now = rospy.Time.now()
    rate = rospy.Rate(10)
    while rospy.Time.now() < now + rospy.Duration.from_sec(2):
      self.cmd_pub.publish(self.vel)
      rate.sleep()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    src = cv_image
    ROI = src[77:581,80:600]
    # ROI = src[175:450,200:450]
    # ROI = src
    hsv = cv.cvtColor(ROI, cv.COLOR_BGR2HSV)

# Threshold of blue in HSV space
    lower_red = np.array([0,0,109])
    upper_red = np.array([179,255,255])

# preparing the mask to overlay
    mask = cv.inRange(hsv, lower_red, upper_red)
    red = cv.bitwise_and(ROI, ROI, mask = mask)
    dst = cv.Canny(red, 50, 200, None, 3)

    # Copy edges to the images that will display the results in BGR
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    lines = cv.HoughLinesP(dst, 1, np.pi / 180, 150, None, 80, 10)
    d_theta = 0

    v = 2.2
    if lines is not None:
        for i in range(0, len(lines)):

            l = lines[i][0]
            slope = (l[3]-l[1])/(l[2]-l[0])
            theta = math.atan(slope)
            theta = round(theta,2)

            if (100<l[0]<280) and (l[2]-l[0])!= 0: #st
              d_theta = (math.pi/2)-theta
              print("center",l[0])
              self.vel.linear.x = v
              if d_theta>0.2:
                self.vel.angular.z = d_theta
                self.cmd_pub.publish(self.vel)
              else:
                self.vel.angular.z = 0
                self.cmd_pub.publish(self.vel)


            if slope<0 and l[0]>100: #right
              d_theta= ((math.pi/2)-theta)
              print("right",l[0])
              self.vel.linear.x = v
              if d_theta>0.2:
                self.vel.angular.z = -0.8*d_theta
                self.cmd_pub.publish(self.vel)
              else:
                self.vel.angular.z = 0
                self.cmd_pub.publish(self.vel)


            elif slope>0 and l[0]<150: #left
              d_theta= ((math.pi/2)-theta)
              print("left",l[0])
              self.vel.linear.x = v
              if d_theta>0.2:
                self.vel.angular.z = 0.8*d_theta
                self.cmd_pub.publish(self.vel)
              else:
                self.vel.angular.z = 0
                self.cmd_pub.publish(self.vel)

            cv.line(ROI, (l[0], l[1]), (l[2], l[3]), (0,0,255), 10, cv.LINE_AA)

    cv.imshow("Image window", ROI)
    cv.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cdst, "bgr8"))
    except CvBridgeError as e:
        print(e)
        pass

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)