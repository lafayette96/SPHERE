import serial
import sys
import matplotlib.pyplot as plt
import numpy as np


try:
	ser = serial.Serial(port='COM5', baudrate=9600, timeout=3)
except OSError as err:
	print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
	# print('\n \t Retrying in 5s')
	sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

print("**Connected to: " + ser.portstr +'**')
	
plt.close('all')
plt.figure()
plt.ion()
plt.show()

data = np.array([])
i=0

while i<100:
	received_text = ser.readline() #read line of serial input
	decoded_text = str(received_text.decode("utf-8")) # decode from bytes to utf-8
	decoded_text = decoded_text.rstrip() # remove trailing newline '\n'
	text_as_list = decoded_text.split(',') #split by the comma
	#print(text_as_list)
	data = np.append(data, text_as_list[0]) #append dataframe with roll value
	# #print(i, data)
	plt.cla() #clear axes
	# axes = plt.gca()
	# axes.set_xlim([0,100])
	# axes.set_ylim([-100,100])
	plt.plot(data)
	plt.pause(0.1)
	i+=1

ser.close()