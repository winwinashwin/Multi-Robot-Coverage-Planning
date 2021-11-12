#!/usr/bin/env python3

import cv2
import numpy
import numpy as np

grids = [
    cv2.imread("cov0.pgm", cv2.IMREAD_GRAYSCALE),
    cv2.imread("cov1.pgm", cv2.IMREAD_GRAYSCALE),
    cv2.imread("cov2.pgm", cv2.IMREAD_GRAYSCALE),
    cv2.imread("cov3.pgm", cv2.IMREAD_GRAYSCALE),
]

original = cv2.imread("map4_small.pgm", cv2.IMREAD_GRAYSCALE)[10:210, 10:210]

merged = 0 * grids[0] + 255

for g in grids:
    merged = cv2.bitwise_and(merged, g)

check = cv2.bitwise_and(merged, original)
arr = np.array(check)
counts = dict(zip(*numpy.unique(arr, return_counts=True)))
print(counts[0] / (counts[0] + counts[254]))

cv2.imshow("merged", check)


cv2.waitKey(0)
