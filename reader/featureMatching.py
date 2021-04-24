import cv2 as cv
import numpy as np


def orbMatch(image, height_pixels,saveImg):
    if saveImg == 1:
        saveImage(image)
    img2 = cv.imread('canny.png', cv.IMREAD_GRAYSCALE)
    img1 = image  # queryImage
    img1 = np.pad(img1, 100, mode='constant')  # orb works better if there is black space around image

    orb = cv.ORB_create(nfeatures=1000) # Initiate ORB detector
    kp1, des1 = orb.detectAndCompute(img1, None) # find the keypoints and descriptors with ORB
    kp2, des2 = orb.detectAndCompute(img2, None)

    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True) # brute force matching
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)
    (xCentroid, yCentroid) = find_centroid(matches, kp1)
    yCentroid = height_pixels - yCentroid

    ##############UNCOMMENT BELOW TO SHOW KEYPOTINTS#######################
    #img1 = cv.drawKeypoints(img1, kp1, None)
    #img2 = cv.drawKeypoints(img2, kp2, None)
    #cv.imshow("Img1", img1)
    #cv.imshow("Img2", img2)

    ##############UNCOMMENT BELOW TO SHOW MATCHES#######################
    #matching_result = cv.drawMatches(img1, kp1, img2, kp2, matches[:100], None)
    #cv.imshow("Matching Result", matching_result)
    #cv.waitKey(0)
    #cv.destroyAllWindows()

    return xCentroid, yCentroid


def find_centroid(matches, kp1):
    x1_sum = 0 # Initialize sums
    y1_sum = 0
    for mat in matches: # Get the matching keypoints for each of the images
        img1_idx = mat.queryIdx
        (x1, y1) = kp1[img1_idx].pt # x - columns, y - rows, Get the coordinates
        x1_sum += x1 # Append to each list
        y1_sum += y1
    samples = len(matches) #If this is an error throws an error which is caught in the main script
    x_centroid = x1_sum / samples - 100  # remember we padded the image with 100 zeros
    y_centroid = y1_sum / samples - 100
    return x_centroid, y_centroid

def saveImage(image): # take the center of an image and pad it with zeros, save it as under the name 'canny'
    img2 = image[30:140, 50:180]  # trainImage
    img2 = np.pad(img2, 100, mode='constant')
    cv.imwrite('canny.png', img2)
