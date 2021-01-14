import numpy as np
import cv2

"""
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
"""


#-----------------------THIS IS THE DIGIT RECOGNITION CODE---------------------------#


#----------INPUT IMAGE AND THRESHOLDING --------------
image = cv2.imread('002.jpg')
hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

#==========================================KERNELS=====================================================
kernel_v = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9), (1,-1))
kernel_r = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5), (1,1))
kernel_o= cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3), (1,1))

#===========UNCOMMENT FROM HERE========

#===========================================OBSTACLES=================================================
obstacle_mask_1 = cv2.inRange(hsv_img, (0,102,86),(40,255,255))
obstacle_mask_2 = cv2.inRange(hsv_img, (164,102,80),(180,255,255))
obstacle_mask = cv2.addWeighted(obstacle_mask_1, 1, obstacle_mask_2, 1, 0)

#apply filtering
obstacle_mask = cv2.erode(obstacle_mask, kernel_r)
obstacle_mask = cv2.dilate(obstacle_mask, kernel_r)
#get contours
contours_o, hierarchy_o = cv2.findContours(obstacle_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
x_o = len(contours_o)

approximated_obstacles = []
#go through contours and apporximate them
for i in range (0, x_o):
	#print (cv2.contourArea(contours[i]))
	approx_contours_o = cv2.approxPolyDP(contours_o[i], 5, True)
	approximated_obstacles.append(approx_contours_o)
	print("obstacle with no of vertices of" , len(approx_contours_o))

print("there are a number of obstacles of", len(approximated_obstacles))

cv2.drawContours(image, approximated_obstacles, -1, (255,255,0), 3)

#===========================================ROBOT=================================================

robot_mask = cv2.inRange(hsv_img, (90,50,50),(140,255,255))
#apply filtering
robot_mask = cv2.erode(robot_mask, kernel_r)
robot_mask = cv2.dilate(robot_mask, kernel_r)
#get contours
contours_r, hierarchy_r = cv2.findContours(robot_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
x_r = len(contours_r)
#print(x_r)
#cv2.imshow("robot mask",robot_mask)

#approximate contours
approximated_robot = []
robot_found = False
#go through contours and apporximate them
for i in range (0, x_r):

	approx_contours_r = cv2.approxPolyDP(contours_r[i], 15, True) #between 4 and 23
	
	if len(approx_contours_r) != 3 :
		continue

	robot_found = True

	approximated_robot.append(approx_contours_r)

print ("robot found!")
print (robot_found)
print(approximated_robot)

cv2.drawContours(image, approximated_robot, -1, (0,255,255), 3)
#cv2.imshow("win1", image)
#key = cv2.waitKey(0)

#================================BORDERS====================================
border_mask = cv2.inRange(image, (0,0,0), (180,180,50))
#apply filtering
kernel_b = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5), (1,-1))
border_mask = cv2.erode(border_mask, kernel_b)
border_mask = cv2.dilate(border_mask, kernel_b)

cv2.imshow("border", border_mask)
#get contours
contours_b, hierarchy_b = cv2.findContours(border_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
x_b = len(contours_b)

approximated_border = []
#go through contours and apporximate them
for i in range (0, x_b):
	if(cv2.contourArea(contours_b[i]) > 7000):
		approx_contours_b = cv2.approxPolyDP(contours_b[i], 5, True)
		approximated_border.append(approx_contours_b)
		print("border with no of vertices of" , len(approx_contours_o))

print("there are a number of borders of", len(approximated_border))

cv2.drawContours(image, approximated_border, -1, (255,100,100), 3)



#========================================VICTIMS=====================================================

victims_mask = cv2.inRange(hsv_img, (30,50,26), (110,255,255))

#--------------FILTERING-----------------------------------
kernel_v_ero = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3), (1,-1)) # 4, 4 and dil 5, 5 - just good
kernel_v_dil = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3), (1,-1))
#cv2.imshow('binary unfiltered',victims_binary)
victims_mask = cv2.erode(victims_mask, kernel_v_ero)
victims_mask = cv2.dilate(victims_mask, kernel_v_dil)
#victims_mask = cv2.GaussianBlur(victims_mask, (5,5),2,2)

#cv2.imshow('binary filtered', victims_mask)
#-------------INVERSION OF BITMAP ---------------------------
inv_mask = cv2.bitwise_not(victims_mask)


#----------------CONTOURS---------------------------
contours_v, hierarchy_v = cv2.findContours(victims_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#print(contours)
x_v = len(contours_v)

#-----------APPROXIMATION OF CONTOURS------------------
approximated_vict = []
approximated_gate = []

bound_rect = []

for i in range (0, x_v):

	approx_contours_v = cv2.approxPolyDP(contours_v[i], 2, True)
	#we check the area, so we can enclose the whole blob, not only the digit - in real world images, the digit may not be a full contour
	#print (cv2.contourArea(contours[i]))
	if len(approx_contours_v) < 7: 

		approximated_gate.append(approx_contours_v)
		print("gate found! With number of vertices ", len(approx_contours_v), " area of: ", cv2.contourArea(contours_v[i]) )
	
	else :
		if cv2.contourArea(contours_v[i]) > 2000:
			approximated_vict.append(approx_contours_v)
			print("victim found! With number of vertices ", len(approx_contours_v), " area of: ", cv2.contourArea(contours_v[i]) )
			#------put rectaangle------
			bound_rect.append(cv2.boundingRect(approx_contours_v))
			x,y,z,t = cv2.boundingRect(approx_contours_v)
			cv2.rectangle(image, (x,y), (x+z, y+t), (255,2,0),2)
			cv2.rectangle(inv_mask, (x,y), (x+z, y+t), (0,0,0),2)


cv2.drawContours(image, approximated_gate, -1, (255,0,100), 3)
cv2.drawContours(image, approximated_vict, -1, (0,255,2), 3)



#filtered = cv2.CreateMat(image.rows, image.cols, CV_8UC3)
#image.copyTo(filtered, victims_mask_inv);

#cv2.imshow('inv', inv_mask)
cv2.imshow("win1", image)

	#------put rectaangle------
	#bound_rect.append(cv2.boundingRect(approx_contours))
	#x,y,z,t = cv2.boundingRect(approx_contours)
	#cv2.rectangle(victims, (x,y), (x+z, y+t), (255,2,0),2)


#-------------INVERSION OF BITMAP ---------------------------
#inv_mask = cv2.bitwise_not(victims_mask)
#cv2.imshow('inv', inv_mask)

#cv2.drawContours(victims, contours, -1, (0,255,0), 3)
#cv2.imshow('test', victims)

#cv2.drawContours(victims, approximated_contours, -1, (0,255,0), 3)
#cv2.drawContours(victims, bound_rect, -1, (70,255,0), 3)
#cv2.imshow('test2', victims)
#cv2.imshow('img',img)
#cv2.imshow('image_hsv', hsv_img)
#cv2.imshow('image_victims', victims_mask)
#cv2.imshow('image_robot', robot_mask)
#cv2.imshow('obstacle_mask', obstacle_mask)

key = cv2.waitKey(0)