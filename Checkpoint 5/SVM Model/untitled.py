from skimage.io import imread, imshow
from skimage.transform import resize
from skimage.feature import hog
from skimage import exposure
import matplotlib.pyplot as plt
import numpy as np
import cv2
import pandas as pd
import os
from sklearn.utils import shuffle
import pickle
from sklearn.svm import SVC
from sklearn import metrics
from sklearn.metrics import confusion_matrix

#################################################################


DIR='./test_images/'      #PATH OF TEST IMAGES
filename='test.txt'		#FILE CONTAINING TEST LABELS
MODEL = joblib.load('/path/to/svm_model.pkl')
MODEL = pickle.load(open('./svm_model.pkl', 'rb'))   

#################################################################


cap = cv2.VideoCapture(0)

def process(image):
	img_copy      = image.copy()
	img_copy      = cv2.cvtColor(img_copy,cv2.COLOR_BGR2GRAY)
	## Blurring and Threshing
	blurred       = cv2.medianBlur(img_copy, 9)
	filter_       = cv2.bilateralFilter(blurred, 5, 75, 75)
	_,thresh = cv2.threshold(filter_,90,255,cv2.THRESH_BINARY)
	
	## Morphological Operations
	element       = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)) 
	dilated       = cv2.dilate(thresh, element, iterations=1)
	
	## Bitwise And Operation
	mask          = 255-dilated
	sample        = mask*img_copy
	
	## Contour Detection using Canny Edge Detection Algorithm
	edged = cv2.Canny(sample,30,200)
	contours, _ = cv2.findContours(edged, 
						  cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	## Find the contour with the maximum area
	try:
		areas = [cv2.contourArea(c) for c in contours]
		if max(areas)>2000:
			max_index = np.argmax(areas)
			cnt=contours[max_index]
		else:
			return image
	except Exception as e:
		return image

	x,y,w,h = cv2.boundingRect(cnt)
	return image[y-20:y+h+10,x-20:x+w+10]

while True:

	# Capture frame-by-frame
	ret, img = cap.read()
	cv2.imshow('Frame',img)

	processed_image=process(img)
	if processed_image.shape[0]==0 or processed_image.shape[1]==0:
		processed_image=img
	resized_img = cv2.resize(processed_image, (410//2,308//2))

	fd,hog_image = hog(resized_img, orientations=9, pixels_per_cell=(8, 8),cells_per_block=(2, 2), visualize=True, channel_axis = 2)

	directions = MODEL.predict([fd])

	print('Prediction', directions)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break 

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()