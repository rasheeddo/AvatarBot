#!/usr/bin/env python3

import numpy as np
from numpy import pi
import socket
import struct
import pickle
import time
import os
import json
os.environ["MAVLINK20"] = "2"
from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil

################################### PORT and SOCKET #############################################

CUBEPILOT_PORT = 5555
cube_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cube_sock.bind(("0.0.0.0", CUBEPILOT_PORT))
cube_sock.setblocking(0)

GPS_PUB_PORT = 8888
gps_pub_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

################################################################################################
vehicle = None
is_vehicle_connected = False

global rover_status
global current_mode

cur_lat = 0.0
cur_lon = 0.0
cur_yaw = 0.0
gps_status = 0

rover_status ={
				"lat" : cur_lat, 
				"lng" : cur_lon,
				"yaw" : cur_yaw, 
				"status" : gps_status}

def vehicle_connect():
	global vehicle, is_vehicle_connected

	if vehicle == None:
		try:
			# if sim.startswith('sitl'):
			# print("Connecting to Ardupilot on SITL...")
			# vehicle = connect('udp:127.0.0.1:14551', wait_ready=True)
			# else:
			print("Connecting to Ardupilot....")
			vehicle = connect('/dev/usb_uart', wait_ready=True, baud=921600)
			
		except Exception as e:
			print(e)
			print("Device might not be found...check the udevrules")
			quit()
			time.sleep(1)

	if vehicle == None:
		is_vehicle_connected = False
		return False
	else:
		is_vehicle_connected = True
		return True

def turn(deg):
	if is_vehicle_connected == True:
		msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
			0b0000101111111111, # type_mask (only speeds enabled)
			0, 0, 0, # x, y, z positions (not used)
			0, 0, 0, # x, y, z velocity in m/s
			0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
			deg*pi/180.0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

		vehicle.send_mavlink(msg)
		# vehicle.flush()
	else:
		print("INFO: Vehicle not connected.")

def goForward(meter):
	if is_vehicle_connected == True:
		msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
			0b0000111111111000, # type_mask (only speeds enabled)
			meter, 0, 0, # x, y, z positions (not used)
			0, 0, 0, # x, y, z velocity in m/s
			0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
			0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
			
		vehicle.send_mavlink(msg)
		# vehicle.flush()
	else:
		print("INFO: Vehicle not connected.")

def goLeft(meter):
	if is_vehicle_connected == True:
		msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
			0b0000111111111000, # type_mask (only speeds enabled)
			0, -meter, 0, # x, y, z positions (not used)
			0, 0, 0, # x, y, z velocity in m/s
			0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
			0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
			
		vehicle.send_mavlink(msg)
		# vehicle.flush()
	else:
		print("INFO: Vehicle not connected.")

def goRight(meter):
	if is_vehicle_connected == True:
		msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
			0b0000111111111000, # type_mask (only speeds enabled)
			0, meter, 0, # x, y, z positions (not used)
			0, 0, 0, # x, y, z velocity in m/s
			0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
			0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
			
		vehicle.send_mavlink(msg)
		# vehicle.flush()
	else:
		print("INFO: Vehicle not connected.")


# Callback to print the location in global frame
def location_callback(self, attr_name, value):
	# global cur_lat, cur_lon
	cur_lat = value.global_frame.lat
	cur_lon = value.global_frame.lon
	# print("cur_lat: %.7f  cur_lon: %.7f" %(cur_lat, cur_lon))
	rover_status['lat'] = cur_lat
	rover_status['lng'] = cur_lon
	
	# print(value.global_frame)

def attitude_callback(self, attr_name, value):
	# global cur_yaw
	cur_yaw = value.yaw
	# print("cur_yaw: %.6f" %cur_yaw)
	rover_status['yaw'] = cur_yaw
	## range is -pi to pi, 0 is north

def gps_callback(self, attr_name, value):
	# global gps_status
	gps_status = value.fix_type
	# print("gps_status: %d" %gps_status)
	rover_status['status'] = gps_status
	# 3 = 3DFix
	# 4 = 3DGPS
	# 5 = rtkFloat
	# 6 = rtkFixed
	## range is -pi to pi, 0 is north

def change_flight_mode(_mode):

	global current_mode

	if _mode != None:
		if current_mode != _mode:
			vehicle.mode = _mode
		current_mode = _mode


############################# Initialized CUBE and P2D Arm ##########################
print("INFO: Connecting to vehicle.")
while (not vehicle_connect()):
	pass
print("INFO: Vehicle connected.")


vehicle.mode = "HOLD"
current_mode = "HOLD"
vehicle.armed= False
# prev_mode = 'NONE'

vehicle.add_attribute_listener('location', location_callback)
vehicle.add_attribute_listener('attitude', attitude_callback)
vehicle.add_attribute_listener('gps_0', gps_callback)

prev_arm_state = 0
pwm_mid = 1527
STR_val = 0.0
THR_val = 0.0
got_data_time = time.time()
while True:
	# startTime = time.time()
	try:
		data, addr = cube_sock.recvfrom(1024)
		# print("data len", len(data))
		data = pickle.loads(data)
		got_data_time = time.time()
	except socket.error:
		last_got_period = time.time() - got_data_time
		# print("Period {:.4f} No data arrived.. STR {:} THR {:}".format(last_got_period, STR_val, THR_val))
		if (last_got_period > 0.5) and (STR_val != 0 or THR_val != 0):
			print("DANGER...gamepad data is piling... STR was {:} THR was {:}".format(STR_val,THR_val))
			STR_val = 0.0
			THR_val = 0.0
			vehicle.channels.overrides['1'] = pwm_mid
			vehicle.channels.overrides['2'] = pwm_mid
		pass
	else:
		print(data)

		if data['ARMED'] != prev_arm_state:
			if data['ARMED'] == 1:
				vehicle.armed = True
			else:
				vehicle.armed = False

		prev_arm_state = data['ARMED']

		change_flight_mode(data['MODE'])

		if current_mode == "GUIDED":
			if data['TURN_DIR'] is not None:
				if data['TURN_DIR'] == "LEFT45":
					turn(-45)
				elif data['TURN_DIR'] == "LEFT90":
					turn(-90)
				elif data['TURN_DIR'] == "RIGHT45":
					turn(45)
				elif data['TURN_DIR'] == "RIGHT90":
					turn(90)
				elif data['TURN_DIR'] == "U180":
					turn(180)

			if data['FORWARD'] != 0:
				goForward(int(data['FORWARD']))
			elif data['LEFT'] != 0:
				goLeft(int(data['LEFT']))
			elif data['RIGHT'] != 0:
				goRight(int(data['RIGHT']))

		elif current_mode == "MANUAL":
			STR_val = data['STR_VAL']
			THR_val = data['THR_VAL']
			steering_pwm = int(round(STR_val*100 + pwm_mid))
			throttle_pwm = int(round(THR_val*150 + pwm_mid))
			vehicle.channels.overrides['1'] = steering_pwm
			vehicle.channels.overrides['2'] = throttle_pwm

	if rover_status:
		# print("rover_status", rover_status)
		rover_status_packet = pickle.dumps(rover_status)
		gps_pub_sock.sendto(rover_status_packet,("127.0.0.1", GPS_PUB_PORT))


	## a little bit of delay, help cpu loads
	time.sleep(0.001)