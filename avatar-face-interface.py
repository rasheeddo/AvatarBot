#!/usr/bin/env python3

import numpy as np
from numpy import pi
import socket
import struct
import pickle
import time
import os
import json
import cv2

################################### PORT and SOCKET #############################################

FACE_PORT = 6666
face_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
face_sock.bind(("0.0.0.0", FACE_PORT))
face_sock.setblocking(0)

################################################################################################

### Load face images ###
blink_image1 = cv2.imread("/home/nvidia/testAvatarFace/blink_seq/blink_seq1.jpg")
blink_image2 = cv2.imread("/home/nvidia/testAvatarFace/blink_seq/blink_seq2.jpg")
blink_image1 = cv2.resize(blink_image1, (640,480))
blink_image2 = cv2.resize(blink_image2, (640,480))

sleep_image1 = cv2.imread("/home/nvidia/testAvatarFace/sleep_seq/sleep_seq1.jpg")
sleep_image2 = cv2.imread("/home/nvidia/testAvatarFace/sleep_seq/sleep_seq2.jpg")
sleep_image3 = cv2.imread("/home/nvidia/testAvatarFace/sleep_seq/sleep_seq3.jpg")
sleep_image4 = cv2.imread("/home/nvidia/testAvatarFace/sleep_seq/sleep_seq4.jpg")
sleep_image1 = cv2.resize(sleep_image1, (640,480))
sleep_image2 = cv2.resize(sleep_image2, (640,480))
sleep_image3 = cv2.resize(sleep_image3, (640,480))
sleep_image4 = cv2.resize(sleep_image4, (640,480))

talk_image1 = cv2.imread("/home/nvidia/testAvatarFace/talk_seq/talk_seq1.jpg")
talk_image2 = cv2.imread("/home/nvidia/testAvatarFace/talk_seq/talk_seq2.jpg")
talk_image1 = cv2.resize(talk_image1, (640,480))
talk_image2 = cv2.resize(talk_image2, (640,480))

normal_image = cv2.imread("/home/nvidia/testAvatarFace/faces/normal.jpg")
look_left_image = cv2.imread("/home/nvidia/testAvatarFace/faces/look_left.jpg")
look_right_image = cv2.imread("/home/nvidia/testAvatarFace/faces/look_right.jpg")
normal_image = cv2.resize(normal_image, (640,480))
look_left_image = cv2.resize(look_left_image, (640,480))
look_right_image = cv2.resize(look_right_image, (640,480))

window_name = "bear"

cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(window_name,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

FACE = 'sleep'
toggle = True
toggle2 = False
lastTime = time.time()
period = 0.0
faceToggle = False


while True:
	# startTime = time.time()
	try:
		data, addr = face_sock.recvfrom(1024)
		# print("data len", len(data))
		face_data = pickle.loads(data)
	except socket.error:
		pass
	else:
		print(face_data)
		faceToggle = face_data['faceToggle']
		FACE = face_data['FACE']


	if faceToggle:
		cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
		cv2.setWindowProperty(window_name,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
		## There would be non blocking part on this loop
		## so I am using toggle and toggle2 flags and timer to animiate the faces like blink, sleep, talk
		if FACE == "sleep":
			if toggle:
				if period < 0.5:
					cv2.imshow(window_name, sleep_image1)
					period = time.time() - lastTime

				else:
					lastTime = time.time()
					period = 0.0
					toggle = False
					toggle2 = True

			if toggle2:
				if period < 0.5:
					cv2.imshow(window_name, sleep_image2)
					period = time.time() - lastTime

				else:
					lastTime = time.time()
					period = 0.0
					toggle = True
					toggle2 = False

		else:

			if FACE == 'talk':
				if toggle:
					if period < 0.1:
						cv2.imshow(window_name, talk_image1)
						period = time.time() - lastTime

					else:
						lastTime = time.time()
						period = 0.0
						toggle = False
						toggle2 = True

				if toggle2:
					if period < 0.1:
						cv2.imshow(window_name, talk_image2)
						period = time.time() - lastTime

					else:
						lastTime = time.time()
						period = 0.0
						toggle = True
						toggle2 = False

			elif FACE == 'left':
				cv2.imshow(window_name, look_left_image)
			elif FACE == 'normal':
				cv2.imshow(window_name, normal_image)
			elif FACE == 'right':
				cv2.imshow(window_name, look_right_image)
			
			# this tis blink
			else :
				if toggle:
					if period < 3.0:
						cv2.imshow(window_name, blink_image1)
						period = time.time() - lastTime

					else:
						lastTime = time.time()
						period = 0.0
						toggle = False
						toggle2 = True

				if toggle2:
					if period < 0.1:
						cv2.imshow(window_name, blink_image2)
						period = time.time() - lastTime

					else:
						lastTime = time.time()
						period = 0.0
						toggle = True
						toggle2 = False

		cv2.waitKey(1)
			
	else:
		cv2.destroyAllWindows()
		toggle = True
		toggle2 = False
		lastTime = time.time()
		period = 0.0
		FACE = 'blink'



	# ## a little bit of delay, help cpu loads
	time.sleep(0.01)