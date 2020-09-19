# Serial communication test

import pygame
import time
import serial
import sys
import PySimpleGUI as sg


def calculate_turn(turn2):
	turn = 10 * turn2 		# [-100, 100]
	#Make sure it is not 3 digit number:
	if turn == 100:
		turn -= 1
	if turn == -100:
		turn += 1
	
	#Prepare a message:
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
	
	# print(mes_turn)	
	ser.write(mes_turn)


def calculate_speed_adj(speed2):
	if speed2 == 0:
		mes_speed = bytes(str('SR00'), 'utf-8')
	elif speed2 > 0 and speed2 <= 3:
		mes_speed = bytes(str('SF04'), 'utf-8')
	elif speed2 > 3 and speed2 <= 6:
		mes_speed = bytes(str('SF07'), 'utf-8')
	elif speed2 > 6:
		mes_speed = bytes(str('SF20'), 'utf-8')
	elif speed2 < 0 and speed2 >= -3:
		mes_speed = bytes(str('SR04'), 'utf-8')
	elif speed2 < 3 and speed2 >= -6:
		mes_speed = bytes(str('SR07'), 'utf-8')
	elif speed2 < 6:
		mes_speed = bytes(str('SR20'), 'utf-8')
	
	
	# print(mes_speed)	
	ser.write(mes_speed)
	
	
def STOP_button_callback():
	ser.write(b'SF00')
	ser.write(b'TL00')
	print('\n\t\t STOPPING THE ROBOT! \n')
	window['-SPEED-'].update(0)
	window['-TURN-'].update(0)


dispatch_dictionary = {'-STOP-':STOP_button_callback}

sg.theme('DarkTeal9')	# Add a touch of color

try:
	ser = serial.Serial(port='COM5', baudrate=9600, timeout=0.1)
except OSError as err:
	print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
	# print('\n \t Retrying in 5s')
	sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

print("**Connected to: " + ser.portstr +'**')


layout = [  [sg.Frame('Steering:',[[	
			 sg.Slider(range=(-10, 10), orientation='h', size=(40, 20), default_value=0, tick_interval=2, key='-TURN-', enable_events=True)]])],
			[sg.Frame('Throttle:',[[
			 sg.Slider(range=(-10, 10), orientation='v', size=(12, 30), default_value=0, tick_interval=2, key='-SPEED-', enable_events=True)]]),
			 sg.Button('STOP', button_color=('white', 'firebrick4'), size=(20,2), key='-STOP-', enable_events=True)],
			[sg.Frame('Controllers:',[[
			 sg.Radio('PID Controller',"RADIO1", default=True),
			 sg.Radio('Fuzzy Controller',"RADIO1"),
			 sg.Radio('Other Controller',"RADIO1"),]])]]

window = sg.Window('SPHERE PC Control', layout)

ser.write(b'SF00')
line = []

while True:
	
	event, values = window.read()
			
	if event == sg.WIN_CLOSED or event == 'Cancel':	# if user closes window or clicks cancel
		ser.write(b'SF00')
		break
	
	# Lookup event in function dictionary
	if event in dispatch_dictionary:
		func_to_call = dispatch_dictionary[event]   # get function from dispatch dictionary
		func_to_call()
	#else:
	#	print('Event {} not in dispatch dictionary'.format(event))
	
	
	speed2 = int(values['-SPEED-']) # [-10, 10]
	turn2 = int(values['-TURN-']) # [-10, 10]

	
	#Prepare and send the message to the robot:
	# SPEED:
	calculate_speed_adj(speed2)
	# TURN:
	calculate_turn(turn2)
	
	# received_text = ser.read(50)
	received_text = ser.readline()
	# print(received_text.decode('utf-8'))
	
	# for c in ser.read():
		# line.append(c)
		# if c == '\n':
			# print("Line: " + ''.join(line))
			# line = []
			# break



window.close()
ser.close()

