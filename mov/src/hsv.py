import cv2 as cv
import numpy as np

file = "/home/ros/catkin_ws/src/mov/src/unknown.png"
img = cv.imread(file)
Gaussian = cv.GaussianBlur(img, (10, 10), 0)
hsv = cv.cvtColor(Gaussian, cv.COLOR_BGR2HSV)

# Threshold of blue in HSV space
lower_white = np.array([0,0,0], dtype=np.uint8)
upper_white = np.array([0,0,255], dtype=np.uint8)

# preparing the mask to overlay
mask = cv.inRange(hsv, lower_white, upper_white)

# The black region in the mask has the value of 0,
# so when multiplied with original image removes all non-blue regions
result = cv.bitwise_and(img, img, mask = mask)

cv.imshow('frame', img)
cv.imshow("blur",Gaussian)
cv.imshow('mask', mask)
cv.imshow('result', result)

cv.waitKey(0)