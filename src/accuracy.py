#!/usr/bin/env python3

import cv2
import rospy
import numpy as np

def histogram_metric (org, img):
    histg_org = cv2.calcHist ([org],[0],None,[256],[0,256])
    histg_img = cv2.calcHist ([img],[0],None,[256],[0,256])

    sum = np.sum (abs (histg_org - histg_img))
    print ("absolute diff:", sum/(256 * 256))

def absolute_diff_metric (org, img):
    rows, cols = org.shape
    sum = np.sum (abs (org - img))
    print ("absolute diff:", sum/(rows * cols * 256))

def average_diff_metric (org, img):
    org = org[::4, ::4]
    img = img[::4, ::4]

    median_org = cv2.medianBlur (org, 5)
    median_img = cv2.medianBlur (img, 5)

    diff = abs (median_img - median_org)
    sum = np.sum (diff)

    rows, cols = org.shape
    print ("average diff metric:", sum/(rows * cols * 256))

if __name__ == '__main__':
    org = cv2.imread ('airsim_images/0.jpg', 0)
    img = cv2.imread ('airsim_images/1.jpg', 0)

    histogram_metric (org, img)
    absolute_diff_metric (org, img)
    average_diff_metric (org, img)
