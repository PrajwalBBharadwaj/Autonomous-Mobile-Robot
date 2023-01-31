from definitions import *
import os
import numpy as np
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import confusion_matrix,accuracy_score
import cv2
from sklearn import metrics

import matplotlib.pyplot as plt
import pickle
from tqdm import tqdm
import csv


MODEL = pickle.load(open(PATH_TO_MODEL+MODEL_NAME, 'rb'))

def load_image_data(DIR,data):
	images=[]
	labels=[]
	filename=[]
	for each in data:
		im=cv2.imread(os.path.join(DIR,each[0]+".png"))
		images.append(im)
		labels.append(int(each[1]))
		filename.append(each[0])
	return images,labels,filename

def process(image):
	img_copy      = image.copy()
	img_copy      = cv2.cvtColor(img_copy,cv2.COLOR_BGR2GRAY)
	blurred       = cv2.medianBlur(img_copy, 9)
	filter_       = cv2.bilateralFilter(blurred, 5, 75, 75)
	_,thresh = cv2.threshold(filter_,90,255,cv2.THRESH_BINARY)
	element       = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)) 
	dilated       = cv2.dilate(thresh, element, iterations=1)
	mask          = 255-dilated
	sample        = mask*img_copy
	edged = cv2.Canny(sample,30,200)
	contours, _ = cv2.findContours(edged, 
						  cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	try:
		areas = [cv2.contourArea(c) for c in contours]
		if max(areas)>200:
			max_index = np.argmax(areas)
			cnt=contours[max_index]
		else:
			return image
	except:
		return image

	x,y,w,h = cv2.boundingRect(cnt)
	return image[y-20:y+h+10,x-20:x+w+10]

def predict(image,MODEL):
	processed_image=process(image)
	if processed_image.shape[1] ==0 or processed_image.shape[0] == 0:
		processed_image = image
	pred=MODEL.predict(cv2.resize(processed_image,(30,30)).reshape(1,-1))
	return pred


f=open(PATH+FILENAME,'r')
reader = csv.reader(f)
test_data= np.array(list(reader))

test_images,test_labels,filename   = load_image_data(PATH,test_data)
y_pred=[]
for image in test_images:
	y_pred.append(predict(image,MODEL))

print("Accuracy:",metrics.accuracy_score(test_labels, y_pred))
print(confusion_matrix(test_labels, y_pred))

