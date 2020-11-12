#!/usr/bin/env python
import cv2

img_file = 'image.jpg'
img_src = cv2.imread(img_file, 1)
img_gray = cv2.cvtColor(img_src, cv2.COLOR_BGR2GRAY)
img_dst = cv2.Canny(img_gray, 0, 100)

cv2.imshow('src', img_src)
cv2.imshow('dst', img_dst)
cv2.waitKey(0)
cv2.destroyAllWindows()
