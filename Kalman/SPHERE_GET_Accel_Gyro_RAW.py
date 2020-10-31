import serial
import time
import sys

def data_acquisition():
	start_time = time.time()
	for i in range(0, 300):
		#bytesToRead = ser.inWaiting()
		print(i)
		received_text = ser.readline()
		f.write(str(received_text.decode("utf-8"))) #str(i+1)+','+
		#time.sleep(0.4)
		i+=1


if __name__ == "__main__":
	try:
		ser = serial.Serial(port='COM5', baudrate=9600, timeout=3)
	except OSError as err:
		print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
		# print('\n \t Retrying in 5s')
		sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

	print("**Connected to: " + ser.portstr +'**')
	
	#f = open('SPHERE_IMU_Measurements_KalmanVSComplementary.txt', 'w+')
	f = open('SPHERE_IMU_Measurements_KalmanVSMadgwick.txt', 'w+')
	start = time.time()
	data_acquisition()
	end = time.time()
	
	print(end-start)
	
	ser.close()