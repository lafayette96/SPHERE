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
	
	print('Turn message: ', mes_turn.decode("utf-8"))	
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
		mes_speed = bytes(str('SR08'), 'utf-8')
	elif speed2 == -2:
		mes_speed = bytes(str('SR12'), 'utf-8')
	elif speed2 == -3:
		mes_speed = bytes(str('SR18'), 'utf-8')
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
	ser.write(b'SF00')
	ser.write(b'TL00')
	
	
def TURN_slider_callback():
	turn2 = int(values['-TURN-']) # [-10, 10] 
	speed2 = int(values['-SPEED-']) # [-10, 10]
	calculate_turn(turn2)
	calculate_speed_adj(speed2)
	
	
def SPEED_slider_callback():
	turn2 = int(values['-TURN-']) # [-10, 10] 
	speed2 = int(values['-SPEED-']) # [-10, 10]
	calculate_turn(turn2)
	calculate_speed_adj(speed2)

	
dispatch_dictionary = {'-STOP-':STOP_button_callback,
					   '-TURN-':TURN_slider_callback,
					   '-SPEED-':SPEED_slider_callback}


sg.theme('DarkTeal9')	# Add a touch of color

try:
	ser = serial.Serial(port='COM5', baudrate=115200)#, bytesize=8, parity='N', stopbits=1, timeout=1)
except OSError as err:
	print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
	sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

print("**Connected to: " + ser.portstr +'**')

#Column layout
col = [  	[sg.Frame('Steering:',[[	
			 sg.Slider(range=(-10, 10), orientation='h', size=(40, 28), default_value=0, tick_interval=2, key='-TURN-', enable_events=True)]])],
			[sg.Frame('Controllers:',[
			 [sg.Radio('No Controller',"RADIO1", key='-NO-', default=True)],
			 [sg.Radio('PID Controller',"RADIO1", key='-PID-')],
			 [sg.Radio('Fuzzy Controller',"RADIO1", key='-FUZZY-')]])],
			[sg.Button('STOP', button_color=('white', 'firebrick4'), size=(20,2), key='-STOP-', enable_events=True)]
	  ]
	  
col2 = [ [sg.Frame('Roll:',[[
		   sg.Text('0.000000', size=(10, 2), font=('Arial', 12), justification='center', key='-PITCHOUTPUT-')]])],
		 [sg.Frame('Pitch:',[[
		   sg.Text('0.000000', size=(10, 2), font=('Arial', 12), justification='center', key='-ROLLOUTPUT-')]])],
		 [sg.Frame('TIM3->CCR1:',[[
		   sg.Text('00', size=(10, 2), font=('Arial', 12), justification='center', key='-TIM3-')]])],
		 [sg.Frame('TIM2->CCR2:',[[
		   sg.Text('00', size=(10, 2), font=('Arial', 12), justification='center', key='-TIM2-')]])],
	   ]

layout = [  [sg.Frame('Throttle:',[[
			 sg.Slider(range=(-10, 10), orientation='v', size=(12, 28), default_value=0, tick_interval=2, key='-SPEED-', enable_events=True)]]),
			 sg.Column(col),
			 sg.Column(col2)],
			[sg.Frame('Terminal:',[[
			 sg.Output(size=(100,25))]])],
			[sg.Button('Update')]
		 ]

window = sg.Window('SPHERE PC Control', layout, element_justification = 'center')

ser.write(b'SF00')

controller_type = 0

while True:
	event, values = window.read(timeout=300)
			
	if event == sg.WIN_CLOSED or event == 'Cancel':	# if user closes window or clicks cancel
		ser.write(b'SF00')
		break
	elif values['-NO-'] == True:
		controller_type = 0
		mes_speed = bytes(str('NR00'), 'utf-8') # if no controller, send message 'Nxxx'
		ser.write(mes_speed)
	elif values['-PID-'] == True:
		controller_type = 1
		mes_speed = bytes(str('PR00'), 'utf-8') # if PID controller, send message 'Pxxx'
		ser.write(mes_speed)
	elif values['-FUZZY-'] == True:
		controller_type = 2
		mes_speed = bytes(str('FR00'), 'utf-8') # if Fuzzy controller, send message 'Fxxx'
		ser.write(mes_speed)
	
	
	# Lookup event in function dictionary
	if event in dispatch_dictionary:
		func_to_call = dispatch_dictionary[event]   # get function from dispatch dictionary
		func_to_call()
	#else:
	#	print('Event {} not in dispatch dictionary'.format(event))
	
	#speed2 = int(values['-SPEED-']) # [-10, 10]
	#turn2 = int(values['-TURN-']) # [-10, 10] 
	
	#Prepare and send the message to the robot:
		# SPEED:
	#calculate_speed_adj(speed2)
	
	# TURN:
	#calculate_turn(turn2)
	
	if controller_type == 0:
		print('No controller enabled\n')
	elif controller_type == 1:
		print('PID controller enabled\n')
	elif controller_type ==2:
		print('Fuzzy controller enabled\n')
	
	ser.flushInput()
	time.sleep(0.01)
	received_text = ser.readline()
	#print(received_text)
	position = received_text.decode("utf-8")
	position_values = position.split(',')
	#print(position_values)
	try:
		roll_val = float(position_values[0])
		pitch_val = float(position_values[1])
		TIM2_val = int(position_values[2])
		TIM3_val = int(position_values[3])
	except(IndexError, ValueError):
		print('\n\n\nERROR OCCURED\n\n\n')
		continue
	print('Pitch:     ', roll_val, '\nRoll:       ', pitch_val, '\nTIM2 reg: ', TIM2_val,'\nTIM3 reg: ',TIM3_val,'\n----------------------------------------------------------------------------------------------------------------')
	
	window['-ROLLOUTPUT-'].update('{:f}'.format(roll_val))
	window['-PITCHOUTPUT-'].update('{:f}'.format(pitch_val))
	window['-TIM3-'].update('{:d}'.format(TIM3_val))
	window['-TIM2-'].update('{:d}'.format(TIM2_val))
	

#window.close()
ser.close()
