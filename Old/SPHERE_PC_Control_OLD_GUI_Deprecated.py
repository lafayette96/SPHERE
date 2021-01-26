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
	
	print('Turn message: ',mes_turn.decode("utf-8"))	
	ser.write(mes_turn)


def calculate_speed_adj(speed2):
	# if speed2 == 0:
		# mes_speed = bytes(str('SR00'), 'utf-8')
	# elif speed2 > 0 and speed2 <= 3:
		# mes_speed = bytes(str('SF04'), 'utf-8')
	# elif speed2 > 3 and speed2 <= 6:
		# mes_speed = bytes(str('SF07'), 'utf-8')
	# elif speed2 > 6:
		# mes_speed = bytes(str('SF20'), 'utf-8')
	# elif speed2 < 0 and speed2 >= -3:
		# mes_speed = bytes(str('SR04'), 'utf-8')
	# elif speed2 < 3 and speed2 >= -6:
		# mes_speed = bytes(str('SR07'), 'utf-8')
	# elif speed2 < 6:
		# mes_speed = bytes(str('SR20'), 'utf-8')
	
	if speed2 == 0:
		mes_speed = bytes(str('SR00'), 'utf-8')
	elif speed2 == 1:
		mes_speed = bytes(str('SF07'), 'utf-8')
	elif speed2 == 2:
		mes_speed = bytes(str('SF10'), 'utf-8')
	elif speed2 == 3:
		mes_speed = bytes(str('SF15'), 'utf-8')
	elif speed2 == 4:
		mes_speed = bytes(str('SF25'), 'utf-8')
	elif speed2 == 5:
		mes_speed = bytes(str('SF30'), 'utf-8')
	elif speed2 == 6:
		mes_speed = bytes(str('SF40'), 'utf-8')
	elif speed2 == 7:
		mes_speed = bytes(str('SF50'), 'utf-8')
	elif speed2 == 8:
		mes_speed = bytes(str('SF60'), 'utf-8')
	elif speed2 == 9:
		mes_speed = bytes(str('SF65'), 'utf-8')
	elif speed2 == 10:
		mes_speed = bytes(str('SF99'), 'utf-8') 
#-------------------------------------------------------
	elif speed2 == -1:
		mes_speed = bytes(str('SR04'), 'utf-8')
	elif speed2 == -2:
		mes_speed = bytes(str('SR07'), 'utf-8')
	elif speed2 == -3:
		mes_speed = bytes(str('SR15'), 'utf-8')
	elif speed2 == -4:
		mes_speed = bytes(str('SR25'), 'utf-8')
	elif speed2 == -5:
		mes_speed = bytes(str('SR30'), 'utf-8')
	elif speed2 == -6:
		mes_speed = bytes(str('SR40'), 'utf-8')
	elif speed2 == -7:
		mes_speed = bytes(str('SR50'), 'utf-8')
	elif speed2 == -8:
		mes_speed = bytes(str('SR55'), 'utf-8')
	elif speed2 == -9:
		mes_speed = bytes(str('SR60'), 'utf-8')
	elif speed2 == -10:
		mes_speed = bytes(str('SR99'), 'utf-8')
	
	print('Speed message: ', mes_speed.decode("utf-8"))	
	ser.write(mes_speed)
	
	
def STOP_button_callback():
	ser.write(b'SF00')
	ser.write(b'TL00')
	print('\n\t\t STOPPING THE ROBOT! \n')
	print(ser.readline())
	window['-SPEED-'].update(0)
	window['-TURN-'].update(0)

	
dispatch_dictionary = {'-STOP-':STOP_button_callback}


sg.theme('DarkTeal9')	# Add a touch of color

try:
	ser = serial.Serial(port='COM5', baudrate=115200)#, bytesize=8, parity='N', stopbits=1, timeout=1)
except OSError as err:
	print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
	sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

print("**Connected to: " + ser.portstr +'**')

#Column layout
col = [  [sg.Frame('Steering:',[[	
			 sg.Slider(range=(-10, 10), orientation='h', size=(40, 28), default_value=0, tick_interval=2, key='-TURN-', enable_events=True)]])],
			[sg.Frame('Controllers:',[
			 [sg.Radio('No Controller',"RADIO1", key='-NO-', default=True)],
			 [sg.Radio('PID Controller',"RADIO1", key='-PID-')],
			 [sg.Radio('Fuzzy Controller',"RADIO1", key='-FUZZY-')]])],
			[sg.Button('STOP', button_color=('white', 'firebrick4'), size=(20,2), key='-STOP-', enable_events=True)]
	  ]

layout = [  [sg.Frame('Throttle:',[[
			 sg.Slider(range=(-10, 10), orientation='v', size=(12, 28), default_value=0, tick_interval=2, key='-SPEED-', enable_events=True)]]),
			 sg.Column(col)],
			[sg.Frame('Terminal:',[[
			 sg.Output(size=(80,20))]])],
			[sg.OK()]
		 ]

window = sg.Window('SPHERE PC Control', layout, element_justification = 'center')

ser.write(b'SF00')

controller_type = 0

while True:
	event, values = window.read()
			
	if event == sg.WIN_CLOSED or event == 'Cancel':	# if user closes window or clicks cancel
		ser.write(b'SF00')
		break
	elif values['-NO-'] == True:
		controller_type = 0
	elif values['-PID-'] == True:
		controller_type = 1
	elif values['-FUZZY-'] == True:
		controller_type = 2
	
	
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
	
	if controller_type == 0:
		print('No controller enabled\n')
	elif controller_type == 1:
		print('PID controller enabled\n')
	elif controller_type ==2:
		print('Fuzzy controller enabled\n')
	
	received_text = ser.readline()
	#print(received_text)
	position = received_text.decode("utf-8")
	position_values = position.split(',')
	print('Roll:      ', position_values[0], '\nPitch:    ', position_values[1], '\nTIM2 reg: ', position_values[2],'----------------------------------------------------------------------------------------------------------------')
	
	

window.close()
ser.close()

'''
# -TURN- and -SPEED- values can range from -10.0 to 10.0
# 'speed' range from 0 to 100
speed = 50 + int(values['-SPEED-'] * 5)
	
# print('Turn value: ',values['-TURN-'], ' Speed value: ',values['-SPEED-'])
# sg.popup('Turn value: ',values['-TURN-'], ' Speed value: ',values['-SPEED-'])
	
	
# def calculate_speed(speed2):
	# speed = 10 * speed2 	# [-100, 100]
	# #Make sure it is not 3 digit number:
	# if speed == 100:
		# speed -= 1
	# if speed == -100:
		# speed += 1
	
	# #Prepare a message:
	# if speed == 0: #if 0, send message not to go
		# mes_speed = bytes(str('SR00'), 'utf-8')
	# elif speed < 0 and speed > -10: # if between -9 and -1, go backwards
		# mes_speed = bytes(str('SR0' + str(abs(speed))), 'utf-8')
	# elif speed <= -10: #if between -99 and -10, turn left
		# mes_speed = bytes(str('SR' + str(abs(speed))), 'utf-8')
	# elif speed > 0 and speed < 10:  # if between 1 and 9, turn right
		# mes_speed = bytes(str('SF0' + str(speed)), 'utf-8')
	# elif speed >= 10: #if between 10 and 99, turn right	
		# mes_speed = bytes(str('SF' + str(speed)), 'utf-8')
	
	# print(mes_speed)	
	# ser.write(mes_speed)
	
print('**STARTING SERVO**')
ser.write(b'SF40')
time.sleep(0.5)
mess = ser.readline()
print(mess)

time.sleep(3)

print('\n**STOPING SERVO**')
ser.write(b'SF00')
time.sleep(0.5)
mess = ser.readline()
print(mess)

	
print('Go to sleep')
time.sleep(2)
print('Wake up')

done = False
counter = 0

print('entering while loop')

while done==False:
	message_turn_byte = b'sf60'
	ser.write(message_turn_byte)
	#time.sleep(2)
	ser.readline()
	
	message_turn_byte = b'sr30'
	ser.write(message_turn_byte)
	#time.sleep(2)
	ser.readline()
	
	counter += 1
	
	if counter == 1:
		done=True

print('Closing')
'''

