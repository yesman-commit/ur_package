#!/usr/bin/env python
import cv2
import time

tl1 = time.time()
t1 = cv2.imread("../src/image.jpg")
t2 = cv2.imread("../src/image_1.jpg")
ten1 = cv2.cvtColor(t1, cv2.COLOR_RGB2GRAY)
ten2 = cv2.cvtColor(t2, cv2.COLOR_RGB2GRAY)
count = 0
for i in range(ten1.shape[0]):
    for j in range(ten1.shape[1]):
        count = count +  (int(ten1[i, j]) - int(ten2[i, j]))
    
print(count)
tl2 = time.time()
print (tl2 - tl1)
