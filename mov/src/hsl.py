import cv2 as cv
import numpy as np

file = "/home/ros/catkin_ws/src/mov/src/unknown.png"
img = cv.imread(file)
hsl = cv.cvtColor(img,cv.COLOR_)