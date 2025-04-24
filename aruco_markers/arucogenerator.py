# AruCo code generator
# Based on: https://github.com/Ritabrata-Chakraborty/ArUco/blob/main/Aruco_Generation.py

import cv2 as cv
import numpy as np
import sys


ARUCO_DICT = {
	"DICT_4X4_50": cv.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv.aruco.DICT_APRILTAG_36h11
}

aruco_type = sys.argv[2]
marker_id = int(sys.argv[1])

window_size = 650
padding = 1

arucoDict = cv.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])

print("ArUCo type '{}' with ID '{}'".format(aruco_type, marker_id))

# Creates an empty image (tag) to draw the ArUco marker on. The image is initialized with zeros.
tag = np.zeros((window_size, window_size, 1), dtype="uint8")
cv.aruco.generateImageMarker(arucoDict, marker_id, window_size, tag, padding)

border = 15

tag = cv.copyMakeBorder(tag, border, border, border, border, cv.BORDER_CONSTANT, value=255)

tag_name = aruco_type + "_" + str(marker_id) + ".png"

cv.imwrite(tag_name, tag)
cv.imshow("ArUCo Tag: {} {}".format(aruco_type, marker_id), tag)

cv.waitKey(0)
cv.destroyAllWindows()