import numpy as np
import cv2


#------------THIS IS THE FILTERING TRIAL AND ERROR CODE--------------------
img = cv2.imread('square_arena_real1.jpg')

#hsv_img = img.clone()

hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

victims_mask = cv2.inRange(hsv_img, (45,50,26), (110,255,255))
robot_mask = cv2.inRange(hsv_img, (90,50,50),(140,255,255))
obstacle_mask_1 = cv2.inRange(hsv_img, (0,102,86),(40,255,255))
obstacle_mask_2 = cv2.inRange(hsv_img, (164,102,80),(180,255,255))
obstacle_mask = cv2.addWeighted(obstacle_mask_1, 1, obstacle_mask_2, 1, 0)

#cv2.imshow('image_victims_raw', victims_mask)
#cv2.imshow('image_robot_raw', robot_mask)
#cv2.imshow('obstacle_mask_raw', obstacle_mask)

kernel_v = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9), (1,-1))
kernel_r = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5), (1,1))
kernel_o= cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3), (1,1))

victims_mask = cv2.erode(victims_mask, kernel_v)
victims_mask = cv2.dilate(victims_mask, kernel_v)
#APPLY GAUSS FILTER FOR TESTING ITS FUNCTIONALITY
victims_mask = cv2.GaussianBlur(victims_mask, (5,5),2,2)

robot_mask = cv2.erode(robot_mask, kernel_r)
robot_mask = cv2.dilate(robot_mask, kernel_r)


obstacle_mask = cv2.erode(obstacle_mask, kernel_r)
obstacle_mask = cv2.dilate(obstacle_mask, kernel_r)

#-----------------------THIS IS THE DIGIT RECOGNITION CODE---------------------------#

#----------INPUT IMAGE AND THRESHOLDING --------------
victims = cv2.imread('nums.jpg')
hsv_img = cv2.cvtColor(victims, cv2.COLOR_BGR2HSV)
victims_binary = cv2.inRange(hsv_img, (45,50,26), (110,255,255))

#--------------FILTERING-----------------------------------
kernel_v_ero = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7), (1,-1))
kernel_v_dil = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5), (1,-1))
cv2.imshow('binary unfiltered',victims_binary)
victims_binary = cv2.erode(victims_binary, kernel_v_ero)
victims_binary = cv2.dilate(victims_binary, kernel_v_dil)
cv2.imshow('binary filtered', victims_binary)

#----------------CONTOURS---------------------------
contours, hierarchy = cv2.findContours(victims_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#print(contours)
x = len(contours)
print(x)

#-----------APPROXIMATION OF CONTOURS------------------
approximated_contours = []
bound_rect = []
for i in range (0, x):
	#we check the area, so we can enclose the whole blob, not only the digit - in real world images, the digit may not be a full contour
	#print (cv2.contourArea(contours[i]))
	if cv2.contourArea(contours[i]) < 5000: 
		continue

	approx_contours = cv2.approxPolyDP(contours[i], 2, True)
	approximated_contours.append(approx_contours)
	#------put rectaangle------
	bound_rect.append(cv2.boundingRect(approx_contours))
	x,y,z,t = cv2.boundingRect(approx_contours)
	cv2.rectangle(victims, (x,y), (x+z, y+t), (255,2,0),2)


#-------------INVERSION OF BITMAP ---------------------------
inv_mask = cv2.bitwise_not(victims_binary)
cv2.imshow('inv', inv_mask)

#cv2.drawContours(victims, contours, -1, (0,255,0), 3)
#cv2.imshow('test', victims)

cv2.drawContours(victims, approximated_contours, -1, (0,255,0), 3)
#cv2.drawContours(victims, bound_rect, -1, (70,255,0), 3)
cv2.imshow('test2', victims)
#cv2.imshow('img',img)
#cv2.imshow('image_hsv', hsv_img)
#cv2.imshow('image_victims', victims_mask)
#cv2.imshow('image_robot', robot_mask)
#cv2.imshow('obstacle_mask', obstacle_mask)

key = cv2.waitKey(0)