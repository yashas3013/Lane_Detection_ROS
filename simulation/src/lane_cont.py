import sys
import math
import cv2 as cv
from cv2 import line
import numpy as np
def main(argv):
    default_file = '/home/ros/catkin_ws/src/simulation/src/Image window_screenshot_11.02.2022.png'
    filename = argv[0] if len(argv) > 0 else default_file
    src = cv.imread(default_file)
    ROI = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    ret,thresh = cv.threshold(ROI,127,255,1) 
    contours,h = cv.findContours(thresh,1,2) 
    for cnt in contours: 
        approx = cv.approxPolyDP(cnt,0.01*cv.arcLength(cnt,True),True) 
        print(len(approx))
        if len(approx)==4: 
            print("square")
            # cv.drawContours(src,[cnt],0,(0,0,255),-1) 
    cv.imshow('img',ROI) 
    cv.waitKey(0) 
    cv.destroyAllWindows() 
if __name__ == "__main__":
    main(sys.argv[1:])