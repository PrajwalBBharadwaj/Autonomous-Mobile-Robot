The function to generate the markers:
# The creation of aruco markers with the cv2 contrib python package (unofficial)
import numpy as np
import cv2
import cv2.aruco as aruco
'''
drawMarker(...)
drawMarker(dictionary, id, sidePixels[, img[, borderBits]]) -> img
'''
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
print(aruco_dict)
# second parameter is id number
# last parameter is total image size
img = aruco.drawMarker(aruco_dict, 7, 700)
cv2.imwrite("test_marker.jpg", img)
cv2.imshow('frame',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
