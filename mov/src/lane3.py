#!/usr/bin/env python
from __future__ import print_function
from dis import dis
import math
from operator import le
from termios import VEOL
import turtle

from matplotlib import image
from  nav_msgs.msg import Odometry
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
    while rospy.Time.now() < now + rospy.Duration.from_sec(6):
      self.cmd_pub.publish(self.vel)
      rate.sleep()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
    self.odom_sub  = rospy.Subscriber("/odom",Odometry,self.callback_odom)
  def to_positive_angle(self,th):
    while True:
        if self.th < 0:
            self.th += 360
        if self.th > 0:
            ans = self.th % 360
            return ans

  def callback_odom(self,data):
    self.x = data.pose.pose.position.x
    self.y = data.pose.pose.position.y
    q1 = data.pose.pose.orientation.x
    q2 = data.pose.pose.orientation.y
    q3 = data.pose.pose.orientation.z
    q4 = data.pose.pose.orientation.w
    q = (q1, q2, q3, q4)
    e = euler_from_quaternion(q)
    self.th = math.degrees(e[2])
    self.th = self.to_positive_angle(self.th)
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    src = cv_image
    # ROI = src[77:581,80:600]
    # ROI = src[175:450,200:450]
    ROI = src
    hsv = cv.cvtColor(ROI, cv.COLOR_BGR2HSV)
# Threshold of blue in HSV space
    lower_red = np.array([0,0,96])
    upper_red = np.array([0,255,255])
# preparing the mask to overlay
    mask = cv.inRange(hsv, lower_red, upper_red)
    red = cv.bitwise_and(ROI, ROI, mask = mask)
    dst = cv.Canny(red, 50, 200, None, 3)
    # Copy edges to the images that will display the results in BGR
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    lines = cv.HoughLinesP(dst, 1, np.pi / 180, 150, None, 80, 10)
    # print(lines)
    # self.vel = Twist()
    d_theta = 0
    poly_vertices = []
    order = [1,2,3,4]
    left_lines = []
    right_lines = []
    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            x1 = l[0]
            y1 = l[1]
            x2 = l[2]
            y2 = l[3]
            cv.line(ROI, (l[0], l[1]), (l[2], l[3]), (0,0,255), 10, cv.LINE_AA) 
            if x1 == x2:
                pass
            else:
                m = (y2-y1)/(x2-x1)
                c = y1-m*x1
                if m<0:
                    left_lines.append((m,c))
                elif m>=0:
                    right_lines.append((m,c))
        print(left_lines,right_lines)
        left_line = np.mean(left_lines, axis=0)
        right_line = np.mean(right_lines,axis=0)
        li = [left_line,right_line]
        # print(li[1])
        if li[0] and li[1] != None:
            for slope,intercept in li:
                rows,cols = ROI.shape[:2]
                y1 = int(rows)
                y2 = int(rows*0.6)
                x1=int((y1-intercept)/slope)
                x2=int((y2-intercept)/slope)
                poly_vertices.append((x1, y1))
                poly_vertices.append((x2, y2))
                cv.line(ROI, (x1,y1), (x2, y2), (0,0,255), 10, cv.LINE_AA)
        poly_vertices = [poly_vertices[i] for i in order]
        cv.fillPoly(ROI, pts = np.array([poly_vertices],'int32'), color = (0,255,0))
        cv.addWeighted(image,0.7,ROI,0.4,0.)


    cv.imshow("Image window", ROI)
    # cv.imshow("fiter",red)
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