import serial
import time
import sys


if __name__ == "__main__":
	try:
		ser = serial.Serial(port='COM5', baudrate=9600, timeout=3)
	except OSError as err:
		print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
		# print('\n \t Retrying in 5s')
		sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

	print("**Connected to: " + ser.portstr +'**')
	
	f = open('SPHERE_IMU_Measurements_Angle_Left_S.txt', 'w+')
	#f = open('SPHERE_IMU_Measurements_KalmanVSMadgwick.txt', 'w+')
	
	turn = 0
	
	while turn < 10:
		print(turn)
		mes_turn = bytes(str('TL' + str(turn)), 'utf-8')
		ser.write(mes_turn)
		time.sleep(1)
		ser.write(mes_turn)
		received_text = ser.readline()
		print(received_text)
		f.write(str(received_text.decode("utf-8")))

		turn +=1
	
	# for turn in range(10,95,5):
		# mes_turn = bytes(str('TR' + str(turn)), 'utf-8')		
		# ser.write(mes_turn)
		# time.sleep(200)
		# ser.write(mes_turn)
		# received_text = ser.readline()
		# f.write(str(received_text.decode("utf-8"))) #str(i+1)+','+
		
	# print(end-start)
	
	ser.close()