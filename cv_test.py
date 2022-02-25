#!/usr/bin/env python

import rospy
import cv2
import numpy as np

img = cv2.imread('/home/dodo/color.png')
print('shape',img.shape)

height, width = img.shape[:2]

imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

lower_blue=(120-10,30,30)
upper_blue=(120+10,255,255)
img_mask=cv2.inRange(imgHSV,lower_blue,upper_blue)

img_result = cv2.bitwise_and(img, img, mask = img_mask)

cv2.imshow('img_origin', img)
cv2.imshow('img_color', img_result)

cv2.waitKey(0)
cv2.destroyAllWindows()
