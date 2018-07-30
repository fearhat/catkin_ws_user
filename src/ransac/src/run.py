#!/usr/bin/env python
import ransac
import cv2
import matplotlib.pyplot as plt
import math
import random
import numpy as np
from scipy.stats import linregress

img_ = cv2.imread("../img/fahrbahn.png", 0)
img = cv2.erode(img_, np.ones((5, 5), np.uint8), iterations=1)
whiteX = np.array([])
whiteY = np.array([])
whitePoints = np.array([])
height, width = img.shape


for i in xrange(height):
    for j in xrange(width):
        if (img.item(i, j) == 255):
            whiteX = np.append(whiteX, j)
            whiteY = np.append(whiteY, i)

whitePoints = zip(whiteX, whiteY)
whitePointsLeft = filter(lambda x: x[0] <= 700, whitePoints)
whitePointsRight = filter(lambda x: x[0] >= 790, whitePoints)
whitePointsMid = filter(lambda x: x[0] > 700 and x[0] < 790, whitePoints)


def ransac(data, margin, iterations):
    res = [0, 0, 0]

    def iterate(data, margin, bestMdl):
        p1 = data[random.randint(1, len(data)) - 1]
        p2 = data[random.randint(1, len(data)) - 1]
        line = [p1, p2]

        # linr = linregress(x=line)
        # print(line)
        # print(linr.slope)
        # m = linr.slope
        # n = linr.intercept
        m = (p2[1] - p1[1]) / (p2[0] - p1[0])
        n = p2[1] - m * p2[0]

        # def fn(x): return m * x + n
        curPt = 0

        for p3 in data:
            d = np.cross(np.subtract(p2, p1), np.subtract(p1, p3)
                         ) / np.linalg.norm(np.subtract(p2, p1))
            dist = abs(d)
            if (dist <= margin):
                curPt += 1

        if (curPt > bestMdl[2]):
            return [m, n, curPt]
        
        return bestMdl
        

    for i in xrange(iterations):
        res = iterate(data, margin, res)
        # print(res)

    return res


def line(m, n):
    return lambda x: int(round(m*x + n))

# t = whitePoints[0] - whitePoints[30]


# plt.scatter(whiteX, whiteY)
resL = ransac(whitePointsLeft, 2, 200)
resR = ransac(whitePointsRight, 2, 200)
fnL = line(resL[0], resL[1])
fnR = line(resR[0], resR[1])

# print(res)

# plt.show()
# print(whitePoints)

lP1 = (0, fnL(0))
lP2 = (width, fnL(width))

rP1 = (0, fnR(0))
rP2 = (width, fnR(width))

cv2.line(img, lP1, lP2, (255, 255, 255), 2)
cv2.line(img, rP1, rP2, (255, 255, 255), 2)

cv2.imshow("Image", img)
key = cv2.waitKey(0) & 0xFF
if key == 27:
    cv2.destroyAllWindows()
