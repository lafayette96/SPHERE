import serial
import time

def data_acquisition():
	start_time = time.time()
	for i in range(0, 550):
		#bytesToRead = ser.inWaiting()
		received_text = ser.readline()
		f.write(str(time.time()-start_time)+','+str(received_text.decode("utf-8"))) #str(i+1)+','+
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
	
	f = open('SPHERE_IMU_Measurements2.txt', 'w+')
	
	data_acquisition()
	
	ser.close()