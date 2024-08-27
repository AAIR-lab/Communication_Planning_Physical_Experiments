#!/usr/bin/env python
import cv2
import numpy as np
from skimage import io, transform
from skimage.transform import hough_line, probabilistic_hough_line
from skimage.feature import canny
import matplotlib.pyplot as plt

# Load the image
""" image = io.imread('planner_scripts/map.pgm', as_gray=True)

# Apply Canny edge detection
edges = canny(image, sigma=0.5, low_threshold=10, high_threshold=50)

# Apply Hough transform to extract lines
lines = transform.probabilistic_hough_line(edges, threshold=10, line_length=10, line_gap=3)

# Loop over all detected lines and extract start and end points
for line in lines:
    p1, p2 = line
    print("Start point: ({}, {})".format(p1[0], p1[1]))
    print("End point: ({}, {})".format(p2[0], p2[1]))
    plt.plot ([p1[1], p2[1]],[p1[0], p2[0]])


plt.show() """

image = cv2.imread('planner_scripts/map.pgm')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# Detect lines using LSD algorithm
lines = cv2.createLineSegmentDetector().detect(blurred)
height = image.shape[0]
conversion_factor = 0.1  # meters/unit

# Loop over all detected lines and extract start and end points
for line in lines[0]:
    x1, y1, x2, y2 = map(int, line[0])
    y1 = height - y1
    y2 = height - y2
    x1 = x1 * conversion_factor
    y1 = y1 * conversion_factor
    x2 = x2 * conversion_factor
    y2 = y2 * conversion_factor
    #p1, p2 = line.ravel()
    #print("Start point: ({}, {})".format(p1[0], p1[1]))
    #print("End point: ({}, {})".format(p2[0], p2[1]))
    plt.plot ([x1,x2], [y1,y2])

plt.show()
