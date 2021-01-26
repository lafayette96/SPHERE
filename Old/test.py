import pygame
import time
import serial
import sys
import PySimpleGUI as sg


try:
	ser = serial.Serial(port='COM5', baudrate=115200)#, bytesize=8, parity='N', stopbits=1, timeout=1)
except OSError as err:
	print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
	sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

print("**Connected to: " + ser.portstr +'**')

while True:
	received_text = ser.readline()
	#print(received_text)
	position = received_text.decode("utf-8")
	position_values = position.split(',')
	roll_val = float(position_values[0])
	pitch_val = float(position_values[1])
	TIM2_val = int(position_values[2])
	print('Roll:      ', roll_val, '\nPitch:    ', pitch_val, '\nTIM2 reg: ', TIM2_val)
	time.sleep(0.1)
	ser.flushInput()