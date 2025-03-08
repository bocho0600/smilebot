from __future__ import print_function
import sys
import os

# Locate file with DFRobot functions
DFRobot_module_directory = "/home/group16/units/egb320/EGB320-Group16"
sys.path.append(DFRobot_module_directory)

import time
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board


class MobilityModule:
	board = Board(1, 0x10)    # RaspberryPi select bus 1, set address to 0x10

	@classmethod
	def board_detect(cls):
		l = cls.board.detecte()
		print("Board list conform:")
		print(l)

	''' print last operate status, users can use this variable to determine the result of a function call. '''
	@classmethod
	def print_board_status(cls):
		if cls.board.last_operate_status == cls.board.STA_OK:
			print("board status: everything ok")
		elif cls.board.last_operate_status == cls.board.STA_ERR:
			print("board status: unexpected error")
		elif cls.board.last_operate_status == cls.board.STA_ERR_DEVICE_NOT_DETECTED:
			print("board status: device not detected")
		elif cls.board.last_operate_status == cls.board.STA_ERR_PARAMETER:
			print("board status: parameter error, last operate no effective")
		elif cls.board.last_operate_status == cls.board.STA_ERR_SOFT_VERSION:
			print("board status: unsupport board framware version")

	# Set constants
	board.set_encoder_enable(board.ALL)                 # Set selected DC motor encoder enable
		#board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
	board.set_encoder_reduction_ratio(board.ALL, 150)   # Set selected DC motor encoder reduction ratio, given on website motor reduction ratio is 100
	board.set_moter_pwm_frequency(1000)                 # Set DC motor pwm frequency to 1000HZ

	## Begin functions. IC facing forward.
	# M1 is RIGHT wheel.  Has encoder and   CW forward
	# M2 is LEFT wheel.   Has encodeer and  CCW forward
	'''!
				@brief motor_movement
				@param id             Motor list, items in range 1 to 2, or id = self.ALL
				@param orientation    Motor orientation, self.CW (clockwise) or self.CCW (counterclockwise)
				@param speed         Motor pwm duty cycle, in range 0 to 100, otherwise no effective
			'''

	@classmethod
	def Forwards(cls, speed): 
		print("Forwards")
		cls.board.motor_movement([cls.board.M1], cls.board.CW, speed)
		cls.board.motor_movement([cls.board.M2], cls.board.CCW, speed)
		ReadSpeed = cls.board.get_encoder_speed(cls.board.ALL)      # Use boadrd.all to get all encoders speed
		print("M1 encoder speed: %d rpm, M2 encoder speed %d rpm" %(ReadSpeed[0], ReadSpeed[1]))

	@classmethod
	def Backwards(cls, speed): 
		print("Begin backing up")
		cls.board.motor_movement([cls.board.M1], cls.board.CCW, speed)
		cls.board.motor_movement([cls.board.M2], cls.board.CW, speed)
		ReadSpeed = cls.board.get_encoder_speed(cls.board.ALL)      # Use boadrd.all to get all encoders speed
		print("M1 encoder speed: %d rpm, M2 encoder speed %d rpm" %(ReadSpeed[0], ReadSpeed[1]))
		print("finito")
		
	@classmethod
	def TurnRight(cls, speed):
		print("Begin right turn")
		cls.board.motor_movement([cls.board.M1], cls.board.CW, speed)
		cls.board.motor_movement([cls.board.M2], cls.board.CW, speed) 
		print("Finished right turn")
		
	@classmethod
	def TurnLeft(cls, speed): 
		print("Begin left turn")
		cls.board.motor_movement([cls.board.M1], cls.board.CCW, speed)
		cls.board.motor_movement([cls.board.M2], cls.board.CCW, speed)
		print("Finished left turn")
		
	@classmethod
	def Turn360(cls, speed): 
		print("Begin 360")
		cls.board.motor_movement([cls.board.M1], cls.board.CW, speed)
		cls.board.motor_movement([cls.board.M2], cls.board.CW, speed)
		print("Finished 360")
		
	@classmethod
	def Stop(cls): 
		cls.board.motor_stop(cls.board.ALL)# stop all DC motor
		print("Motors stopped")

	@classmethod
	def Move(cls, linear_velocity, angular_velocity):
			'''
			@brief Move the robot based on linear and angular velocity
			@param linear_velocity    Speed of the robot's forward/backward motion (positive for forward, negative for backward)
			@param angular_velocity   Rate of rotation (positive for right turn, negative for left turn)
			'''
			# Convert velocities to motor speed ranges
			max_speed = 255  # 255 is the maximum speed (because DC write (pwm) is 255)
			left_motor_speed = linear_velocity - angular_velocity
			right_motor_speed = linear_velocity + angular_velocity

			# Ensure the speeds are within the allowable range
			left_motor_speed = max(min(left_motor_speed, max_speed), -max_speed)
			right_motor_speed = max(min(right_motor_speed, max_speed), -max_speed)
			# print("The left motor speed is ",left_motor_speed )
			# print("The right motor speed is ",right_motor_speed )
			# Determine the movement direction based on speed values//7
			if left_motor_speed > 0:
					cls.board.motor_movement([cls.board.M1], cls.board.CW, abs(left_motor_speed))
			elif left_motor_speed < 0:
					cls.board.motor_movement([cls.board.M1], cls.board.CCW, abs(left_motor_speed))
			else:
					cls.board.motor_stop([cls.board.M1])
			
			if right_motor_speed > 0:
					cls.board.motor_movement([cls.board.M2], cls.board.CCW, abs(right_motor_speed))
			elif right_motor_speed < 0:
					cls.board.motor_movement([cls.board.M2], cls.board.CW, abs(right_motor_speed))
			else:
					cls.board.motor_stop([cls.board.M2])
			
			# Print the status for debugging
			# print(f"Left motor speed: {left_motor_speed}")
			# print(f"Right motor speed: {right_motor_speed}")

# # Example usage

# #(100, 0)  # Move forward with a slight right turn
# #speed = 100
# time.sleep(1)
# Move(50, 0)  # Move forward with a slight right turn
# #Move(-80,0)
# #Backwards(60)
# time.sleep(30)
MobilityModule.Stop()

