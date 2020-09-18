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


def get_gamepad_event():
	"""Just print out some event infomation when the gamepad is used."""
	while 1:
		events = get_gamepad()
		for event in events:
			if event.ev_type == "Absolute" and event.code == "ABS_RZ":
				if event.state > 0 and event.state <= 100:
					ser.write(b'SF04')
				elif event.state > 100 and event.state <= 200:
					ser.write(b'SF07')
				elif event.state > 200:
					ser.write(b'SF20')
				else:
					ser.write(b'SF00')
			if event.ev_type == "Absolute" and event.code == "ABS_Z":
				if event.state > 0 and event.state <= 100:
					ser.write(b'SR04')
				elif event.state > 100 and event.state <= 200:
					ser.write(b'SR07')
				elif event.state > 200:
					ser.write(b'SR20')
				else:
					ser.write(b'SF00')
			if event.ev_type == "Absolute" and event.code == "ABS_X":
				compute_turn_value(event.state)
				
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
