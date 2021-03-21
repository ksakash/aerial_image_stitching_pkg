#!/usr/bin/env python3

import cv2
import numpy as np
import time
import sys

img1_path = ''
img2_path = ''

img1 = cv2.imread (img1_path, 1)
img2 = cv2.imread (img2_path, 1)

if img1 is None or img2 is None:
    print ("ERROR!!: Unable to read one of the images")
    sys.exit (-1)

gray1 = cv2.cvtColor (img1, cv2.COLOR_BGR2GRAY)
_, mask1 = cv2.threshold (gray1, 1, 255, cv2.THRESH_BINARY)

gray2 = cv2.cvtColor (img2, cv2.COLOR_BGR2GRAY)
_, mask2 = cv2.threshold (gray2, 1, 255, cv2.THRESH_BINARY)

bd = cv2.line_descriptor.BinaryDescriptor_createBinaryDescriptor ()

keylines1, descriptors1 = bd.detect (gray1, mask1, False, False)
keylines2, descriptors2 = bd.detect (gray2, mask2, False, False)

lbd_octave1 = []
lbd_octave2 = []
