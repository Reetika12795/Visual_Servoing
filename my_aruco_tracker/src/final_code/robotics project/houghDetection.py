import numpy as np
import cv2
import math

def main(img):

	#masked = cv2. 
	# image processing 
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	blur = cv2.GaussianBlur(gray,(5,5),cv2.BORDER_DEFAULT)
	# apply image thresholding
	img_edge = cv2.Canny(blur,220,255)

	img_dil = cv2.dilate(img_edge,np.ones((3,3)),iterations=1)

	# apply Hough Line Transformation
	lines = cv2.HoughLinesP(img_dil, 1, np.pi/180, 30, maxLineGap=200)

	#lines is a vector of vector of vectors that contains the position x and y
	#of two points of a line 

	#print(lines)
	# Plot detected lines
	ordre  = 1
	temp = None
	if lines is None: # there's no lines in the image
		print("No lignes detected")
	else:
		#start counting the left and right lines 
		gaucheLines = 0
		droiteLines = 0
		for line in lines:
			x1, y1, x2, y2 = line[0] #get the position of lines top points
			L = math.sqrt(abs(x2 - x1)**2 + abs(y2 - y1)**2) # compute the length of the line			
			alpha = math.acos(abs(x2-x1)/(0.00001+L)) *180/math.pi # angle of orientation
			
			if (alpha > 30 and alpha < 55): #if the lines are in this range of orientation
				# drow the line 
				cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)
				if ( x1 < 600 or x2 < 600):
					#if it's in the left of the image
					cv2.putText(img, 'gauche', (x1, y1) , cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 2, cv2.LINE_AA)
					gaucheLines = gaucheLines + 1
				else:				
					cv2.putText(img, 'droite', (x1, y1) , cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 2, cv2.LINE_AA)
					droiteLines = droiteLines + 1
		if gaucheLines < 4 and droiteLines > 4:
			# if we have more right lines we have to go to left
			ordre = 2
			print("va a gauche")
			print(gaucheLines)
		elif droiteLines < 4 and gaucheLines > 4:
			# if we have more left lines we have to go to right
			ordre = 3
			print(droiteLines)
			print("va a droite")
		else:
			# go straight
			ordre = 1
			print("tout droit! ")	
	cv2.imshow('frame',img) 
 
	return ordre