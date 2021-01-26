#import inputs
import time
import serial
import sys
import math
from inputs import get_gamepad
# import pygame

'''
Using R2 button:
ev_type == "Absolute"
code == "ABS_RZ"
state - how much we press the button (0-255)

Using L2 button:
ev_type == "Absolute"
code == "ABS_Z"
state - how much we press the button (0-255)

Using left joystick:
ev_type == "Absolute"
code == ABS_X and ABS_Y
state - how much we tilt the joystick (ABS_ X and ABS_Y: -32768 to 32767)

Using buttons:
ev_type == "Key"
code == BTN_NORTH  - triangle 
code == BTN_SOUTH  - x
code == BTN_WEST  - square 
code == BTN_EAST  - circle 

'''


def compute_turn_value(state):
# state in (-32768, 32767) - integer
# turn  in (-99, 99) - integer

	turn = math.floor(state/331)
		
	print(turn)
	
	if turn == 0: #if 0, send message not to turn
		mes_turn = bytes(str('TR00'), 'utf-8')
	elif turn < 0 and turn > -10: # if between -9 and -1, turn left
		mes_turn = bytes(str('TL0' + str(abs(turn))), 'utf-8')
	elif turn <= -10: #if between -99 and -10, turn left
		mes_turn = bytes(str('TL' + str(abs(turn))), 'utf-8')
	elif turn > 0 and turn < 10:  # if between 1 and 9, turn right
		mes_turn = bytes(str('TR0' + str(turn)), 'utf-8')
	elif turn >= 10: #if between 10 and 99, turn right	
		mes_turn = bytes(str('TR' + str(turn)), 'utf-8')
	
	ser.write(mes_turn)
	print(mes_turn)


def get_gamepad_event():
	"""Just print out some event infomation when the gamepad is used."""
	while True:
		events = get_gamepad()
		for event in events:
			if event.ev_type == "Absolute" and event.code == "ABS_RZ":
				print(event.state)
				if event.state > 0 and event.state <= 100:
					ser.write(b'SF15')
				elif event.state > 100 and event.state <= 200:
					ser.write(b'SF30')
				elif event.state > 200:
					ser.write(b'SF90')
				else:
					ser.write(b'SF00')
			if event.ev_type == "Absolute" and event.code == "ABS_Z":
				print(event.state)
				if event.state > 0 and event.state <= 100:
					ser.write(b'SR15')
				elif event.state > 100 and event.state <= 200:
					ser.write(b'SR30')
				elif event.state > 200:
					ser.write(b'SR90')
				else:
					ser.write(b'SF00')
			if event.ev_type == "Absolute" and event.code == "ABS_X":
				compute_turn_value(event.state)
			if event.ev_type == "Key":
				if event.code == "BTN_SOUTH" and event.state == 1: 
					mes_turn = bytes(str('NR00'), 'utf-8')
				elif event.code == "BTN_WEST" and event.state == 1:
					mes_turn = bytes(str('PR00'), 'utf-8')
				elif event.code == "BTN_EAST" and event.state == 1:
					mes_turn = bytes(str('FR00'), 'utf-8')
				ser.write(mes_turn)
				print(mes_turn)
		
		# Print IMU measurements
		received_text = ser.readline()
		#print(received_text)
		
		# for event in events:
			# print(event.ev_type, event.code, event.state)


try:
	ser = serial.Serial(port='COM5', baudrate=9600) # , timeout=2)
except OSError as err:
	print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
	# print('\n \t Retrying in 5s')
	sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

print("**Connected to: " + ser.portstr +'**')

ser.write(b'SF00')

get_gamepad_event()

ser.close()
