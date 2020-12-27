#!/usr/bin/env python3

import numpy as np
from numpy import pi
import socket
import struct
import pickle
import time
import os
from Palletizer2DOF import *

################################### PORT and SOCKET #############################################

ROBOTARM_PORT = 7777
arm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
arm_sock.bind(("0.0.0.0", ROBOTARM_PORT))
arm_sock.setblocking(0)

ARM_PUB_PORT = 8899
arm_pub_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

################################################################################################

arm_status = {
				"depth" : 350,
				"height" : 0
}

# print("Start robot arm worker")
arm_device = "/dev/u2d2"
p2d = Palletizer(arm_device)
p2d.RobotTorqueOn()
p2d.GoHome(2000)

finishedTime = 1200
x_target = 350
z_target = 0
prev_depth = x_target
prev_height = z_target
prev_x = x_target
prev_z = z_target
## This flag will let the code not execute wastely
## so we can help cpu computation as less as possible
robot_done = False
unlock = True
all_done = False
while True:

	try:
		data, addr = arm_sock.recvfrom(1024)
		# print("data len", len(data))
		parse_data = pickle.loads(data)
		unlock = True
		all_done = False
		robot_done = False
	except socket.error:
		pass
	else:
		print(parse_data)
		x_target = parse_data["ARM_X"]
		z_target = parse_data["ARM_Z"]


	if not all_done:
		robot_done = p2d.GotoPointInTime(x_target, z_target, finishedTime)
		# print("robot_done ", robot_done)
		if robot_done:
			if unlock:
				# print("here")
				## the socket recv doesn't get as a stream
				## so we can add some delay here for better read XZ
				time.sleep(0.4)
				cur_x, cur_z = p2d.GetXZ()
				unlock = False
				all_done = True
			arm_status['depth'] = int(cur_x)
			arm_status['height'] = int(cur_z)
			# print("depth %d  height %d " %(rover_status['arm']['depth'], rover_status['arm']['height']))
			# Done = False
			
		else:
			## if the robot is not done, then let update the slider as the target at first
			## otherwise the slider will jump around with old and new values
			# lock.acquire()
			arm_status['depth'] = int(x_target)
			arm_status['height'] = int(z_target)
			# lock.release()



	if (prev_depth != arm_status['depth']) or (prev_height != arm_status['height']):
		# print("rover_status", rover_status)
		arm_status_packet = pickle.dumps(arm_status)
		arm_pub_sock.sendto(arm_status_packet,("127.0.0.1", ARM_PUB_PORT))

	prev_depth = arm_status['depth']
	prev_height = arm_status['height']
	prev_x = x_target
	prev_z = z_target
	## a little bit of delay, help cpu loads
	time.sleep(0.001)
