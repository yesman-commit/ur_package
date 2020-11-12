#!/usr/bin/env python
import cv2
import glob
import matplotlib.pyplot as plt
import numpy as np

import os

akaze = cv2.AKAZE_create() 

filepath_train = glob.glob("../src/image.jpg")
filepath_query = glob.glob("../src/image_1.jpg")

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
df = pd.DataFrame(columns=['x_query','y_query','x_train','y_train','Distance','img_index'])  

for num in range((len(filepath_train))):
    img_train = cv2.imread(filepath_train[num])
    img_query = cv2.imread(filepath_query[num])
    kp_train, des_train = akaze.detectAndCompute(img_train, None)
    kp_query, des_query = akaze.detectAndCompute(img_query, None)     
    img_train_key = cv2.drawKeypoints(img_train, kp_train, None, flags=4)
    img_query_key = cv2.drawKeypoints(img_query, kp_query, None, flags=4)

    matches = bf.match(des_train, des_query)    
    matches = sorted(matches, key = lambda x:x.distance)

    for i in range(len(matches)):
        df.loc["Matches"+str(i)] = [kp_train[matches[i].queryIdx].pt[0],kp_train[matches[i].queryIdx].pt[1],kp_query[matches[i].trainIdx].pt[0],kp_query[matches[i].trainIdx].pt[1],matches[i].distance,matches[i].imgIdx]
    df.to_csv("Matches"+os.path.split(filepath_train[num])[1][:-4]+"_"+os.path.split(filepath_query[num])[1][:-4]+".csv")

    img_match = cv2.drawMatches(img_train, kp_train, img_query, kp_query, matches[:10], None, flags=2)
    cv2.imwrite(os.path.split(filepath_train[num])[1][:-4]+"_"+os.path.split(filepath_query[num])[1][:-4]+"_keypoint.png",img_match)