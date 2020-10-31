import matplotlib.pyplot as plt 
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
import serial
import time 
import sys


def animate(i, sample, roll, pitch):
	# Acquire data from serial
	line = ser.readline() #ascii
	print(line)
	line_as_list = line.split(b',')
	
	# Add new values to the list
	i = 1
	sample.append(i)
	roll.append(line_as_list[0])
	pitch.append(line_as_list[1])

	# Draw x and y lists
	ax.clear()
	ax.plot(sample, roll, label="Roll")
	ax.plot(sample, pitch, label="Pitch")
	
	# Format plot
	plt.xticks(rotation=45, ha='right')
	plt.subplots_adjust(bottom=0.30)
	plt.title('This is how I roll...')
	plt.ylabel('Relative frequency')
	plt.legend()
	plt.axis([1, None, 0, 1.1]) #Use for arbitrary number of trials
	#plt.axis([1, 100, 0, 1.1]) #Use for 100 trial demo
	
	#Increment sample counter
	#counter += 1


if __name__ == "__main__":
	print('** Trying to connect to device...**')

	try:
		ser = serial.Serial(port='COM5', baudrate=9600, timeout=3)
	except OSError as err:
		print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
		# print('\n \t Retrying in 5s')
		sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

	print("**Connected to: " + ser.portstr +'**')
	
	# Create figure for plotting
	fig = plt.figure()
	ax = fig.add_subplot(1, 1, 1)
	sample = [] # store timestamps here (triels, n)
	roll = [] #store roll values here
	pitch = [] #store pitch values here
	
	# Set up plot to call animate() function periodically
	ani = animation.FuncAnimation(fig, animate, fargs=(sample, roll, pitch), interval=100)
	plt.show()
	
	
	
	
	