import serial
import time
import sys

def prepare_message(turn):
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
		
	return(mes_turn)


if __name__ == "__main__":
	try:
		ser = serial.Serial(port='COM5', baudrate=9600, timeout=3)
	except OSError as err:
		print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
		# print('\n \t Retrying in 5s')
		sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

	print("**Connected to: " + ser.portstr +'**')
	
	f = open('SPHERE_IMU_Measurements_Dynamic.txt', 'w+')
	#f = open('SPHERE_IMU_Measurements_KalmanVSMadgwick.txt', 'w+')
	
	turn = 0
	start = time.time()
	counter = 0
	
	while True:
		elapsed = time.time() - start
		print(int(elapsed))
		
		if int(elapsed) == 2:
			ser.write(bytes(str('TR90'), 'utf-8'))
		elif int(elapsed) == 4:
			ser.write(bytes(str('TR50'), 'utf-8'))
		elif int(elapsed) == 6:
			ser.write(bytes(str('TR00'), 'utf-8'))
		else:
			print('nosz kurwa')
		
		time.sleep(1)
		


	
	# for turn in range(10,95,5):
		# mes_turn = bytes(str('TR' + str(turn)), 'utf-8')		
		# ser.write(mes_turn)
		# time.sleep(200)
		# ser.write(mes_turn)
		# received_text = ser.readline()
		# f.write(str(received_text.decode("utf-8"))) #str(i+1)+','+
		
	# print(end-start)
	
	ser.close()