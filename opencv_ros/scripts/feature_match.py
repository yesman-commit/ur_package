#!/usr/bin/env python
import cv2
import numpy as np

img_file_1 = '../src/image.jpg'
img_file_2 = '../src/image_1.jpg'

img_src_1 = cv2.imread(img_file_1, 1)
img_src_2 = cv2.imread(img_file_2, 1)

img_gray_1 = cv2.cvtColor(img_src_1, cv2.COLOR_BGR2GRAY)
img_gray_2 = cv2.cvtColor(img_src_2, cv2.COLOR_BGR2GRAY)

img_zero_1 = np.zeros((img_src_1.shape[0], img_src_1.shape[1], 3), np.uint8)
img_zero_2 = np.zeros((img_src_2.shape[0], img_src_2.shape[1], 3), np.uint8)

corners_1 = cv2.goodFeaturesToTrack(img_gray_1, 1000, 0.1, 5)
corners_2 = cv2.goodFeaturesToTrack(img_gray_2, 1000, 0.1, 5)
fe_1 = 0
fe_2 = 0

for i in corners_1:
    x, y = i.ravel()
    cv2.circle(img_zero_1, (x, y), 1, (255, 255, 255), 2)
    fe_1 = fe_1 + 1

for i in corners_2:
    x, y = i.ravel()
    cv2.circle(img_zero_2, (x, y), 1, (255, 255, 255), 2)
    fe_2 = fe_2 + 1

cv2.imshow("src", img_zero_1)
cv2.imshow("dst", img_zero_2)


print(fe_1)
print(fe_2)



detector = cv2.AKAZE_create()
kpts1, desc1 = detector.detectAndCompute(img_zero_1, None)
kpts2, desc2 = detector.detectAndCompute(img_zero_2, None)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
matches = matcher.match(desc1, desc2)

h1 = img_zero_1.shape[0]
w1 = img_zero_1.shape[1]
h2 = img_zero_2.shape[0]
w2 = img_zero_2.shape[1]

img_dst = np.zeros((max(h1, h2), w1 + w2, 3), np.uint8)

cv2.drawMatches(img_src_1, kpts1, img_src_2, kpts2, matches, img_dst)
cv2.imwrite("output.jpg", img_dst)

cv2.imshow('out', img_dst)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''print(kpts1)
print(kpts2)'''

print(matches[0].distance)
print(matches[0].trainIdx)
print(matches[0].queryIdx)
print(matches[0].imgIdx)
print(kpts1[0].pt)
print(kpts1[0].size)
print(kpts1[0].angle)
print(kpts1[0].response)
print(kpts1[0].octave)
print(kpts1[0].class_id)
print('query: ', kpts1[matches[0].queryIdx].pt)
print('train: ', kpts2[matches[0].trainIdx].pt)
