import cv2 as cv
import numpy as np


def nothing(x):
    while 0:
        print(x)


def getEdges(image, adjustThreshold):
    image8 = np.uint8(image)
    if adjustThreshold == 1:
        canny = cannyAdjust(image8)
    else:
        canny = cv.Canny(image8, 75, 250)  # must be cast as unit8 to work
    return canny


def cannyAdjust(image):
    image8 = np.uint8(image)
    canny = np.zeros(image8.shape)
    cv.namedWindow('canny')
    cv.createTrackbar('low', 'canny', 0, 1000, nothing)
    cv.createTrackbar('high', 'canny', 0, 1000, nothing)
    while 1:
        cv.imshow('canny', canny)
        k = cv.waitKey(1) & 0xFF
        if k == 27:
            break
        lowThreshold = cv.getTrackbarPos('low', 'canny')
        highThreshold = cv.getTrackbarPos('high', 'canny')
        canny = cv.Canny(image8, lowThreshold, highThreshold)
    cv.destroyAllWindows()
    return canny
