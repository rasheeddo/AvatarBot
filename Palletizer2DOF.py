import numpy as np 
import math as m
from dynamixel_sdk import * 

class Palletizer:
	def __init__(self, device):

		#### Kinematics parameters ####
		self.L2 = 350		# mm
		self.L3 = 350		# mm

		# servo offset is the offset from robot angle ex. when robot ang. = 0 then servo ang. = 90
		self.SERVO_OFFSET = 90.0		# deg

		#### Workspace parameters ####
		# psi is an angle between drive link3 and idle link2
		self.psi_min = 24				# deg
		self.psi_max = 165				# deg
		# upper triangle constant angle is a constant angle to maintain end effector parallel
		self.UT_const_ang = 48.89		# deg
		# lamda is an angle between main link3 and upper triangle
		self.lamda_min = 15				# deg
		self.lamda_max = 168			# deg

		self.servo2_min = 64
		self.servo2_max = 205

		self.servo3_min = 56
		self.servo3_max = 210

		self.x_min = 92.0
		self.x_max = 685.0

		self.z_min = -475.0
		self.z_max = 550.0

		#### Target variable ####
		self.servo2_target = None
		self.servo3_target = None
		self.x_target = None
		self.z_target = None

		self.prev_deg2 = None
		self.prev_deg3 = None

		self.prev_raw_deg2 = 2000
		self.prev_raw_deg3 = 2000


		#### Servo parameters ####
		############## Control table address ##############
		self.ADDR_PRO_MODEL_NUMBER       		 = 0
		self.ADDR_PRO_DRIVE_MODE         		 = 10
		self.ADDR_PRO_OPERATING_MODE     		 = 11
		self.ADDR_PRO_CURRENT_LIMIT      		 = 38
		self.ADDR_PRO_ACCELERATION_LIMIT 		 = 40
		self.ADDR_PRO_VELOCITY_LIMIT     		 = 44
		self.ADDR_PRO_TORQUE_ENABLE      		 = 64
		self.ADDR_PRO_POSITION_D_GAIN    		 = 80
		self.ADDR_PRO_POSITION_I_GAIN    		 = 82
		self.ADDR_PRO_POSITION_P_GAIN    		 = 84
		self.ADDR_PRO_FEEDFORWARD_2nd_GAIN		 = 88
		self.ADDR_PRO_FEEDFORWARD_1st_GAIN 		 = 90
		self.ADDR_PRO_GOAL_CURRENT       		 = 102
		self.ADDR_PRO_GOAL_VELOCITY      		 = 104
		self.ADDR_PRO_PROFILE_ACCELERATION  	 = 108		# VELOCITY BASED PROFILE
		self.ADDR_PRO_PROFILE_VELOCITY   		 = 112		# VELOCITY BASED PROFILE
		self.ADDR_PRO_PROFILE_ACCELERATION_TIME  = 108		# TIME BASED PROFILE
		self.ADDR_PRO_PROFILE_TIME_SPAN			 = 112      # TIME BASED PROFILE
		self.ADDR_PRO_GOAL_POSITION      		 = 116
		self.ADDR_PRO_MOVING             		 = 122
		self.ADDR_PRO_MOVING_STATUS       		 = 123
		self.ADDR_PRO_PRESENT_CURRENT    		 = 126 
		self.ADDR_PRO_PRESENT_POSITION   		 = 132

		############## Data Byte Length ##############
		self.LEN_PRO_GOAL_POSITION       	     = 4
		self.LEN_PRO_PRESENT_POSITION            = 4
		self.LEN_PRO_GOAL_CURRENT				 = 2
		self.LEN_PRO_PRESENT_CURRENT             = 2
		self.LEN_PRO_POS_TIME                    = 12		

		############### Operating Mode Number ##############
		self.CURRENT_CONTROL                     = 0
		self.POSITION_CONTROL                    = 3 		# Default
		self.CURRENT_BASED_POSITION_CONTROL      = 5

		self.TIME_BASED                          = 4
		self.VELOCITY_BASED                      = 0

		############## Protocol version ##############
		self.PROTOCOL_VERSION            = 2.0              

		# ID
		# self.DXL1_ID                      = 1                          
		self.DXL2_ID                      = 2                             
		self.DXL3_ID                      = 3                            
		# self.DXL4_ID                      = 4
		self.BAUDRATE                    = 57600             # Dynamixel default self.BAUDRATE : 57600
		self.DEVICENAME                  = device    # Check which port is being used on your controller
															 # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
		self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
		self.TORQUE_DISABLE              = 0                 # Value for disabling the torque


		#### Initialization ####

		# Initialize PortHandler instance
		# Set the port path
		# Get methods and members of PortHandlerLinux or PortHandlerWindows
		self.portHandler = PortHandler(self.DEVICENAME)

		# Initialize PacketHandler instance
		# Set the protocol version
		# Get methods and members of Protocol1PacketHandler or Protocoself.L2PacketHandler
		self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

		############## Open port ##############
		if self.portHandler.openPort():
			print("Succeeded to open the port")
		else:
			print("Failed to open the port")
			print("Press any key to terminate...")
			getch()
			quit()

		############## Set port BAUDRATE ##############
		if self.portHandler.setBaudRate(self.BAUDRATE):
			print("Succeeded to change the BAUDRATE")
		else:
			print("Failed to change the BAUDRATE")
			print("Press any key to terminate...")
			getch()
			quit()

		# Initialize GroupSyncWrite instance for Goal Position
		self.groupSyncWritePosition = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PRO_GOAL_POSITION, self.LEN_PRO_GOAL_POSITION)

		# Initialize GroupSyncRead instace for Present Position
		# self.groupSyncReadPosition = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)

		# Initialize GroupSyncWrite instance for Goal Current
		# self.groupSyncWriteCurrent = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PRO_GOAL_CURRENT, self.LEN_PRO_GOAL_CURRENT)

		# Initialize GroupSyncRead instace for Present Current
		# self.groupSyncReadCurrent = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)

		# Initialize GroupSyncRWrite instace for Time-based profile 
		self.groupSyncWritePositionInTime = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PRO_PROFILE_ACCELERATION, self.LEN_PRO_POS_TIME)

		# Add parameter storage for Dynamixel#2 present position value
		# self.dxl_addparam_result = self.groupSyncReadPosition.addParam(self.DXL2_ID)
		# if self.dxl_addparam_result != True:
		# 	print("Initialize: ERROR")
		# 	print("[ID:%03d] groupSyncRead addparam failed" % self.DXL2_ID)
		# 	quit()

		# # Add parameter storage for Dynamixel#3 present position value
		# self.dxl_addparam_result = self.groupSyncReadPosition.addParam(self.DXL3_ID)
		# if self.dxl_addparam_result != True:
		# 	print("Initialize: ERROR")
		# 	print("[ID:%03d] groupSyncRead addparam failed" % self.DXL3_ID)
		# 	quit()

		# # Add parameter storage for Dynamixel#2 present current value
		# self.dxl_addparam_result = self.groupSyncReadCurrent.addParam(self.DXL2_ID)
		# if self.dxl_addparam_result != True:
		# 	print("Initialize: ERROR")
		# 	print("[ID:%03d] groupSyncRead addparam failed" % self.DXL2_ID)
		# 	quit()

		# # Add parameter storage for Dynamixel#3 present current value
		# self.dxl_addparam_result = self.groupSyncReadCurrent.addParam(self.DXL3_ID)
		# if self.dxl_addparam_result != True:
		# 	print("Initialize: ERROR")
		# 	print("[ID:%03d] groupSyncRead addparam failed" % self.DXL3_ID)
		# 	quit()		

		if not self.GetOperatingMode(self.DXL2_ID) == 3:
			self.SetOperatingMode(self.DXL2_ID,self.POSITION_CONTROL)
			self.SetOperatingMode(self.DXL3_ID,self.POSITION_CONTROL)

		self.SetDrivingMode(self.DXL2_ID,self.TIME_BASED)
		self.SetDrivingMode(self.DXL3_ID,self.TIME_BASED)

		############## Position PID/FF gains ##############
		## with a camera like Realsense D435 at end-effector
		## I found that gentle PID like 500,0,4000 doesn't perform too aggressive,
		## and keep the structure stiff enough (less wobbling)
		P_Gain2 = 500    #800 default
		I_Gain2 = 0     #0 default
		D_Gain2 = 4000   #4700 default

		P_Gain3 = 500    #800 default
		I_Gain3 = 0     #0 default
		D_Gain3 = 4000   #4700 default

		FF1_Gain2 = 0	#100
		FF2_Gain2 = 0	#50

		FF1_Gain3 = 0	#100
		FF2_Gain3 = 0	#50

		self.SetPID(self.DXL2_ID, P_Gain2, I_Gain2, D_Gain2)
		self.SetPID(self.DXL3_ID, P_Gain3, I_Gain3, D_Gain3)

		self.SetFFGain(self.DXL2_ID, FF1_Gain2, FF2_Gain2)
		self.SetFFGain(self.DXL3_ID, FF1_Gain3, FF2_Gain3)


	def map(self, val, in_min, in_max, out_min, out_max):	  

		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def SetDrivingMode(self,ID,Base):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_DRIVE_MODE, Base)

		self.DriveMode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_DRIVE_MODE)

		if self.DriveMode == 4:
			print("Motor " + str(ID) + " is in Driving Mode : Time-Based Profile")
		elif self.DriveMode == 0:
			print("Motor " + str(ID) + " is in Driving Mode : Velocity-Based Profile")
		else:
			print("Motor " + str(ID) + " is in Driving Mode : Unknown Drive Mode...")

	def GetOperatingMode(self,ID):
		present_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_OPERATING_MODE)

		return present_mode

	def SetOperatingMode(self,ID,MODE):

		## Must set to torque off before 

		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_OPERATING_MODE, MODE)

		present_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_OPERATING_MODE)
		if present_mode == 0:
		    # Current (Torque) Control Mode
			print("Motor " + str(ID) + " Operating Mode is Torque Control")
		elif present_mode == 3:
		    # Position Control Mode
			print("Motor " + str(ID) + " Operating Mode is Position Control")
		elif present_mode == 5:
		    # Current-based Position Control Mode
			print("Motor " + str(ID) + " Operating Mode is Current-based Position Control")
		else:
			print("In other Mode that didn't set!")

	def SetCurrentLimit(self,ID,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 6.8779]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 5.5064]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 1193], [Range(ampere) : 0 ~ 3.2092]

		# UnitMX106 = 3.36 
		# UnitXM540 = 2.69
		# UnitXM430 = 2.69

		# Write ComValue
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_CURRENT_LIMIT,ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		# Read and confirm input value
		dxl_current_limit, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_CURRENT_LIMIT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		#CurLimit = dxl_current_limit*(SetUnit/1000.0)
		print("Current Limit of ID " + str(ID) + " [ComValue]: " + str(dxl_current_limit))
		#print("Current Limit 1 [Ampere] : %f" %CurLimit)

	def SetGoalCurrent(self,ID,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]

		# user input is in ComValue unit

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_GOAL_CURRENT, ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

	def ReadCurrent(self,ID):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		'''
		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69
		'''
		dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PRESENT_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		com_signed_value = np.int16(dxl_present_current)

		#PresentCur = com_signed_value*(SetUnit/1000.0)
		# print("ReadCurrent of ID " + str(ID) + " [ComValue_signed]: " + str(com_signed_value))
		#print("Present current1 [Ampere]: %f" %PresentCur)

		return com_signed_value

	def SetTimeBaseProfile(self,ID,set_Tf,set_Ta):
		######################### Set Velocity / Acceleration Profile in Time-Base ##############################
		if self.DriveMode == 4:
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PROFILE_ACCELERATION_TIME, int(set_Ta))
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PROFILE_TIME_SPAN, int(set_Tf))
			print("Finish Time of Motor " + str(ID) + " : " + str(set_Tf))
			print("Acceleration time of Motor " + str(ID) + " : " + str(set_Ta))
			print("--------------------------------")
		else:
			print("TIME PRFL ERROR : DriveMode is invalid")

	def INV(self,px,pz):

		alpha = np.arctan2(pz,px)
		L = np.sqrt(px**2 + pz**2)
		#from cosine law
		gam = np.arccos(-((L**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3)))
		# print("gam: ", np.degrees(gam))
		#from sine law
		beta = np.arcsin(self.L3*np.sin(gam)/L)

		theta3 = -(np.pi - gam)
		theta2 = alpha + beta

		deg2 = np.degrees(theta2)
		deg3 = np.degrees(theta3)

		if m.isnan(float(deg2))  or m.isnan(float(deg3)):
			deg2 = self.prev_deg2
			deg3 = self.prev_deg3
			print("INV ERROR: Nan velue")

		self.prev_deg2 = deg2
		self.prev_deg3 = deg3

		return deg2, deg3

	def FWD(self,deg2,deg3):

		rad2 = np.radians(deg2)
		rad3 = np.radians(deg3)

		# p2x = self.L2*np.cos(rad2)
		# p2z = self.L2*np.sin(rad2)
		# print("p2x: ", p2x)
		# print("p2z: ", p2z)

		p3x = self.L2*np.cos(rad2) + self.L3*np.cos(rad2+rad3)
		p3z = self.L2*np.sin(rad2) + self.L3*np.sin(rad2+rad3)
		# print("p3x: ", p3x)
		# print("p3z: ", p3z)

		return p3x,p3z #[p2x,p2z] ,[p3x,p3z]

	def RobotAngToServoAng(self,deg2,deg3):
		servo2 = deg2 + self.SERVO_OFFSET
		# For the Pallertizer, when servo2 moves, robot deg3 also got changed 
		servo3 = -deg3 - deg2 + self.SERVO_OFFSET

		return servo2, servo3

	def ServoAngToRobotAng(self,servo2, servo3):
		deg2 = servo2 - self.SERVO_OFFSET
		# For the Pallertizer, when servo2 moves, robot deg3 also got changed  
		deg3 = -(deg2 + servo3 - self.SERVO_OFFSET)

		return deg2, deg3

	def RunServo(self,ServoDeg2,ServoDeg3):

		servo_ang2 = self.map(ServoDeg2, 0.0, 360.0, 0, 4095)
		servo_ang3 = self.map(ServoDeg3, 0.0, 360.0, 0, 4095)

		dxl2_goal_position = int(servo_ang2)
		dxl3_goal_position = int(servo_ang3)

		##### Delta Robot Safety Working Range #####
		
		param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position))]
		param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(dxl3_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl3_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl3_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl3_goal_position))]

		# Add Dynamixel#2 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePosition.addParam(self.DXL2_ID, param_goal_position2)
		if dxl_addparam_result != True:
			print("RunServo: ERROR")
			print("[ID:%03d] groupSyncWrite addparam failed" % self.DXL2_ID)
			self.portHandler.closePort()
			quit()

		# Add Dynamixel#3 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePosition.addParam(self.DXL3_ID, param_goal_position3)
		if dxl_addparam_result != True:
			print("RunServo: ERROR")
			print("[ID:%03d] groupSyncWrite addparam failed" % self.DXL3_ID)
			self.portHandler.closePort()
			quit()

		# Syncwrite goal position
		dxl_comm_result = self.groupSyncWritePosition.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("RunServo: ERROR")
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

		# Clear syncwrite parameter storage
		self.groupSyncWritePosition.clearParam()

	def RunServoInTime(self,inputDeg2,t1_2,t3_2,inputDeg3,t1_3,t3_3):

		servo_ang2 = self.map(inputDeg2, 0.0, 360.0, 0, 4095)
		servo_ang3 = self.map(inputDeg3, 0.0, 360.0, 0, 4095)

		dxl2_goal_position = int(servo_ang2)
		dxl3_goal_position = int(servo_ang3)

		t1_2 = int(t1_2)
		t3_2 = int(t3_2)
		t1_3 = int(t1_3)
		t3_3 = int(t3_3)

		time_position2 = [DXL_LOBYTE(DXL_LOWORD(t1_2)), 
						DXL_HIBYTE(DXL_LOWORD(t1_2)),
						DXL_LOBYTE(DXL_HIWORD(t1_2)), 
						DXL_HIBYTE(DXL_HIWORD(t1_2)),
						DXL_LOBYTE(DXL_LOWORD(t3_2)), 
						DXL_HIBYTE(DXL_LOWORD(t3_2)),
						DXL_LOBYTE(DXL_HIWORD(t3_2)), 
						DXL_HIBYTE(DXL_HIWORD(t3_2)),
						DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)), 
						DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)),
						DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)), 
						DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position))]

		time_position3 = [DXL_LOBYTE(DXL_LOWORD(t1_3)), 
						DXL_HIBYTE(DXL_LOWORD(t1_3)),
						DXL_LOBYTE(DXL_HIWORD(t1_3)), 
						DXL_HIBYTE(DXL_HIWORD(t1_3)),
						DXL_LOBYTE(DXL_LOWORD(t3_3)), 
						DXL_HIBYTE(DXL_LOWORD(t3_3)),
						DXL_LOBYTE(DXL_HIWORD(t3_3)), 
						DXL_HIBYTE(DXL_HIWORD(t3_3)),
						DXL_LOBYTE(DXL_LOWORD(dxl3_goal_position)), 
						DXL_HIBYTE(DXL_LOWORD(dxl3_goal_position)),
						DXL_LOBYTE(DXL_HIWORD(dxl3_goal_position)), 
						DXL_HIBYTE(DXL_HIWORD(dxl3_goal_position))]

		# Add Dynamixel#2 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePositionInTime.addParam(self.DXL2_ID, time_position2)
		if dxl_addparam_result != True:
			print("RunServoInTime: ERROR")
			print("[ID:%03d] groupSyncWrite addparam failed" %self.DXL2_ID)
			self.portHandler.closePort()
			quit()
		# Add Dynamixel#3 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePositionInTime.addParam(self.DXL3_ID, time_position3)
		if dxl_addparam_result != True:
			print("RunServoInTime: ERROR")
			print("[ID:%03d] groupSyncWrite addparam failed" %self.DXL3_ID)
			self.portHandler.closePort()
			quit()

		# Syncwrite goal position
		dxl_comm_result = self.groupSyncWritePositionInTime.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("RunServoInTime: ERROR")
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

		# Clear syncwrite parameter storage
		self.groupSyncWritePositionInTime.clearParam()

	def ReadAngle(self):

		# # Syncread present position
		# dxl_comm_result = self.groupSyncReadPosition.txRxPacket()
		# if dxl_comm_result != COMM_SUCCESS:
		# 	print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

		# # Check if groupsyncread data of Dynamixel#2 is available
		# dxl_getdata_result = self.groupSyncReadPosition.isAvailable(self.DXL2_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		# if dxl_getdata_result != True:
		# 	print("ReadAngle: ERROR")
		# 	print("[ID:%03d] groupSyncRead getdata failed" % self.DXL2_ID)
		# 	#quit()

		# # Check if groupsyncread data of Dynamixel#2 is available
		# dxl_getdata_result = self.groupSyncReadPosition.isAvailable(self.DXL3_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		# if dxl_getdata_result != True:
		# 	print("ReadAngle: ERROR")
		# 	print("[ID:%03d] groupSyncRead getdata failed" % self.DXL3_ID)
		# 	#quit()

		# # Get Dynamixel#2 present position value
		# dxl_present_position2 = self.groupSyncReadPosition.getData(self.DXL2_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		# # Get Dynamixel#3 present position value
		# dxl_present_position3 = self.groupSyncReadPosition.getData(self.DXL3_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)

		dxl_present_position2, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_PRESENT_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadAngle: ERROR")
			print("ID2 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			# self.portHandler.closePort()
			# quit()
			print("Use prev_raw_deg2")
			dxl_present_position2 = self.prev_raw_deg2

		elif dxl_error != 0:
			print("ReadAngle: ERROR")
			print("ID2 %s" % self.packetHandler.getRxPacketError(dxl_error))
			print("Use prev_raw_deg2")
			dxl_present_position2 = self.prev_raw_deg2

		else:
			self.prev_raw_deg2 = dxl_present_position2

		
		dxl_present_position3, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_PRESENT_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadAngle: ERROR")
			print("ID3 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			# self.portHandler.closePort()
			# quit()
			print("Use prev_raw_deg3")
			dxl_present_position3 = self.prev_raw_deg3

		elif dxl_error != 0:
			print("ReadAngle: ERROR")
			print("ID3 %s" % self.packetHandler.getRxPacketError(dxl_error))
			print("Use prev_raw_deg3")
			dxl_present_position3 = self.prev_raw_deg3
		else:
			self.prev_raw_deg3 = dxl_present_position3



		deg2 = self.map(dxl_present_position2, 0.0, 4095.0, 0.0, 360.0)
		deg3 = self.map(dxl_present_position3, 0.0, 4095.0, 0.0, 360.0)


		return deg2, deg3

	def TorqueOn(self,ID):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)

		if dxl_comm_result != COMM_SUCCESS:
			print("ERROR TorqueOn: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("ERROR TorqueOn: %s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			print("Torque is enable")

	def TorqueOff(self,ID):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)

		if dxl_comm_result != COMM_SUCCESS:
			print("ERROR TorqueOff: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("ERROR TorqueOff: %s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			print("Torque is disable")

	def RobotTorqueOn(self):
		self.TorqueOn(self.DXL2_ID)
		self.TorqueOn(self.DXL3_ID)

	def RobotTorqueOff(self):
		self.TorqueOff(self.DXL2_ID)
		self.TorqueOff(self.DXL3_ID)

	def SetTimeBaseProfile(self,ID,set_Tf,set_Ta):
		######################### Set Velocity / Acceleration Profile in Time-Base ##############################
		if self.DriveMode == 4:
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PROFILE_ACCELERATION_TIME, int(set_Ta))
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PROFILE_TIME_SPAN, int(set_Tf))
			print("Finish Time of Motor " + str(ID) + " : " + str(set_Tf))
			print("Acceleration time of Motor " + str(ID) + " : " + str(set_Ta))
			print("--------------------------------")
		else:
			print("TIME PRFL ERROR : DriveMode is invalid")

	def SetPID(self,ID,set_P_Gain,set_I_Gain,set_D_Gain):

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		print("Position P Gain of ID" + str(ID) + ": " + str(set_P_Gain))
		print("Position I Gain of ID" + str(ID) + ": " + str(set_I_Gain))
		print("Position D Gain of ID" + str(ID) + ": " + str(set_D_Gain))
		print("------------------------------")

	def SetFFGain(self,ID,set_FF1_Gain,set_FF2_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

		print("Feedforward 1st Gain of ID " + str(ID) + ": " + str(set_FF1_Gain))
		print("Feedforward 2nd Gain of ID " + str(ID) + ": " + str(set_FF2_Gain))
		print("------------------------------") 

	def GoHome(self, finished_time):

		servo2 = 180.0
		servo3 = 90.0
		
		t1 = finished_time/2		#400
		t3 = finished_time		#800

		robotDeg = self.ServoAngToRobotAng(servo2,servo3)

		self.prev_deg2 = robotDeg[0]
		self.prev_deg3 = robotDeg[1]

		self.RunServoInTime(servo2,t1,t3,servo3,t1,t3)
		#self.IsAllStop()

	def GetXYZ(self):

		ReadAng = self.ReadAngle()

		robotAngle2, robotAngle3 = self.ServoAngToRobotAng(ReadAng[0], ReadAng[1])
		XYZ = self.FWD(robotAngle2,robotAngle3)
		x = XYZ[0]
		z = XYZ[1]

		return x,z, robotAngle2, robotAngle3

	def GetXZ(self):

		ReadAng = self.ReadAngle()

		robotAngle2, robotAngle3 = self.ServoAngToRobotAng(ReadAng[0], ReadAng[1])
		XYZ = self.FWD(robotAngle2,robotAngle3)
		x = XYZ[0]
		z = XYZ[1]

		return x,z

	def ServoInRange(self, servo2, servo3):
		phi = 180.0 - (servo2-self.SERVO_OFFSET) - (servo3-self.SERVO_OFFSET)
		psi = 180.0 - phi

		lamda = self.UT_const_ang - (-(servo3-self.SERVO_OFFSET))

		if psi < self.psi_min or psi > self.psi_max:
			print("ServoInRange ERROR: psi angle is %f deg, linkage may collide" %psi)
			return False
		else:
			if lamda < self.lamda_min or lamda > self.lamda_max:
				print("ServoInRange ERROR: lamda angle is %f deg, linkage may collide" %lamda)
				return False

			if servo2 < self.servo2_min and servo2 > self.servo2_max:
				print("ServoInRange ERROR: servo2 angle is %f deg, linkage may collide" %servo2)
				return False
			
			if servo3 < self.servo3_min and servo3 > self.servo3_max:
				print("ServoInRange ERROR: servo3 angle is %f deg, linkage may collide" %servo3)
				return False

			return True

	def XZInRange(self,x,z):

		if (z < self.z_min or z > self.z_max):
			return False
		else:
			if x < self.x_min or x > self.x_max:
				return False
			else:
				return True

	def KinematicsCheck(self):
		try:
			while True:

				ServoAng = self.ReadAngle()
				print("ServoAng2: %f" %ServoAng[0])
				print("ServoAng3: %f" %ServoAng[1])
				phi = 180.0 - (ServoAng[0]-self.SERVO_OFFSET) - (ServoAng[1]-self.SERVO_OFFSET)
				psi = 180.0 - phi
				print("psi : %f" %(psi))

				print("ServoAng2 w/offset: %f" %(ServoAng[0]-self.SERVO_OFFSET))
				print("ServoAng3 w/offset: %f" %(ServoAng[1]-self.SERVO_OFFSET))
				lamda = 48.89 - (-(ServoAng[1]-self.SERVO_OFFSET))
				print("lamda : %f" %(lamda))

				flag = self.ServoInRange(ServoAng[0],ServoAng[1])

				XYZ = self.GetXYZ()
				print("===      Present XYZ from FWD      ===")
				print("X: %f" %XYZ[0])
				print("Z: %f" %XYZ[1])
				print("===          Robot Angle         ===")
				print("RobotAng2: %f" %XYZ[2])
				print("RobotAng3: %f" %XYZ[3])

				X = XYZ[0]
				Z = XYZ[1]
				RobotAng2 = XYZ[2]
				RobotAng3 = XYZ[3]
				INVDEG = self.INV(X,Z)
				print("===     Present Angle from INV     ===")
				print("Deg2: %f" %INVDEG[0])
				print("Deg3: %f" %INVDEG[1])
				print("===  Error Angle from calculation  ===")
				errorANG2 = RobotAng2 - INVDEG[0]
				errorANG3 = RobotAng3 - INVDEG[1]
				print("Error Deg2: %f" %errorANG2)
				print("Error Deg3: %f" %errorANG3)
				print("-------------------------------------")
				time.sleep(0.5)
		except(KeyboardInterrupt, SystemExit):
			print("End program...")

	def checkSameTarget(self, x, z):
		if self.x_target == x and self.z_target == z:
			
			# print("checkSameTarget:")
			# print("x", x)
			# print("y", y)
			# print("z", z)
			return True
		else:
			return False

	def IsMoving(self,ID):
		Moving, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_MOVING)
		# print("ID %d  %s" %(ID, Moving))
		return Moving

	def GotoPointInTime(self,x,z,finishedTime):

		#print("checkSameTarget")
		samePoint = self.checkSameTarget(x,z)
		#print("XYZOutRange")
		InRange = self.XZInRange(x,z)

		if InRange:
			if not samePoint:

				GoalPos = self.INV(x, z)
				GoalServo = self.RobotAngToServoAng(GoalPos[0],GoalPos[1])

				if self.ServoInRange(GoalServo[0], GoalServo[1]):
					pass
				else:
					print("GotoPointInTime ERROR: servo out range")
					return True

				## Update latest target that received
				self.x_target = x
				self.z_target = z

			
				# Time Base Profile
	    		# Finish time
				t3_2 = finishedTime
				t3_3 = finishedTime
				t1_2 = t3_2/2
				t1_3 = t3_3/2

				self.halfTime = finishedTime/2000  # second
				#print("RunServoInTime")
				self.RunServoInTime(GoalServo[0],t1_2,t3_2, GoalServo[1],t1_2,t3_2)
				self.startRunTime = time.time()
				#time.sleep(finishedTime/2000)

				return False			# to let main loop knows that it doesn't reach target point

				# Bypass stop motion
				#self.IsAllStop()

			else:

				twoIsDone = not self.IsMoving(2)
				threeIsDone = not self.IsMoving(3)
				self.checkRunTime = time.time()
				period = (self.checkRunTime - self.startRunTime)
				#print("period ", period)
				if ((period > self.halfTime) and twoIsDone and threeIsDone):
					return True
				else:
					return False

		else:
			print("ERROR GotoPointInTime: Out of workspace")
			return True

	def GotoByJoystick(self,x,z):

		InRange = self.XZInRange(x,z)

		if InRange:

			GoalPos = self.INV(x, z)
			GoalServo = self.RobotAngToServoAng(GoalPos[0],GoalPos[1])

			if self.ServoInRange(GoalServo[0], GoalServo[1]):
				pass
			else:
				print("GotoByJoystick ERROR: servo out range")
				return 

			## Update latest target that received
			self.x_target = x
			self.z_target = z

			## write only position target
			self.RunServo(GoalServo[0], GoalServo[1])
			#self.RunServoInTime(GoalServo[0],t1_2,t3_2, GoalServo[1],t1_2,t3_2)

		else:
			print("ERROR GotoByJoystick: Out of workspace")
			return 

