#!/usr/bin/env python
import cv2
import numpy

image_file = 'image.jpg'
img_src = cv2.imread(image_file, 1)
img_gray = cv2.cvtColor(img_src, cv2.COLOR_BGR2GRAY)
img_dst = img_src.copy()
img_zero = numpy.zeros((img_src.shape[0], img_src.shape[1], 3), numpy.uint8)

corners = cv2.goodFeaturesToTrack(img_gray, 1000, 0.1, 5)
ten = 0

for i in corners:
    x,y = i.ravel()
    cv2.circle(img_zero, (x, y), 1, (0, 0, 255), 2)
    ten = ten + 1

cv2.imshow('src', img_src)
cv2.imshow('dst', img_zero)
cv2.waitKey(0)
print(ten)
cv2.destroyAllWindows()