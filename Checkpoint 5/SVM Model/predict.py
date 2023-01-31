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
MODEL = pickle.load(open('./svm_model.pkl', 'rb'))   

#################################################################

def process(image,file):
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


df=pd.read_csv(DIR+filename,names=['file','label'])
files=df['file'].to_list()
labels=df['label'].to_list()
lookup = {str(files[i]): labels[i] for i in range(len(files))}
print(lookup)

test_images=[]
test_labels=[]

for i,file in enumerate(os.listdir(DIR)):
	if 'png' in file or 'jpg' in file:#and ('1000' in file or '2000' in file or '213' in file):
		print('in')
		image=cv2.imread(DIR+file)	
		processed_image=process(image,file)
		if processed_image.shape[0]==0 or processed_image.shape[1]==0:
			processed_image=image
		resized_img = cv2.resize(processed_image, (410//2,308//2))
		fd,hog_image = hog(resized_img, orientations=9, pixels_per_cell=(8, 8),cells_per_block=(2, 2), visualize=True, channel_axis=2)
		test_images.append(fd)
		print(file)
		test_labels.append(lookup[file[:-4]])
		#test_labels.append(lookup)
	
	
test_images=np.array(test_images)/255
y_pred=MODEL.predict(test_images)

print("Accuracy:",metrics.accuracy_score(test_labels, y_pred))
print(confusion_matrix(test_labels, y_pred))