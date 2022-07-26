# Python code for Multiple Color Detection


""" this will detect where the post-it notes are so that they can be trasnformed into a new camera when they move"""

import numpy as np
import cv2


# Capturing video through webcam
webcam = cv2.VideoCapture(0)

webcam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
webcam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
webcam.set(cv2.CAP_PROP_EXPOSURE, 50)
#webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
#webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
webcam.set(cv2.CAP_PROP_FPS, 15)

# Start a while loop
while(1):
	
	# Reading the video from the
	# webcam in image frames
	_, imageFrame = webcam.read()

	# Convert the imageFrame in
	# BGR(RGB color space) to
	# HSV(hue-saturation-value)
	# color space
	hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

	# Set range for red color and
	# define mask
	red_lower = np.array([136, 87, 111], np.uint8)
	red_upper = np.array([180, 255, 255], np.uint8)
	red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

	
	# Morphological Transform, Dilation
	# for each color and bitwise_and operator
	# between imageFrame and mask determines
	# to detect only that particular color
	kernal = np.ones((5, 5), "uint8")
	
	# For red color
	red_mask = cv2.dilate(red_mask, kernal)
	res_red = cv2.bitwise_and(imageFrame, imageFrame,
							mask = red_mask)

	# Creating contour to track red color
	contours, hierarchy = cv2.findContours(red_mask,
										cv2.RETR_TREE,
										cv2.CHAIN_APPROX_SIMPLE)
	
	centres_list = []
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(3500>area > 3000):
			x, y, w, h = cv2.boundingRect(contour)
			centres_list.append([x,y])
			imageFrame = cv2.rectangle(imageFrame, (x, y),
									(x + w, y + h),
									(0, 0, 255), 2)
			
			cv2.putText(imageFrame, "Red Colour", (x, y),
						cv2.FONT_HERSHEY_SIMPLEX, 1.0,
						(0, 0, 255))

	if (len(centres_list) == 6):
		with open("calibrations.txt","w") as file:
			file.write(str(centres_list))
			print(centres_list)

	# Program Termination
	cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
	if cv2.waitKey(10) & 0xFF == ord('q'):
		webcam.release()
		cv2.destroyAllWindows()
		break
