import serial
import time

def data_acquisition():
	start_time = time.time()
	for i in range(0, 200):
		#bytesToRead = ser.inWaiting()
		print(i)
		received_text = ser.readline()
		f.write(str(received_text.decode("utf-8"))) 
		# To add timestamp add at the beginning: str(time.time()-start_time)+','+
		#time.sleep(0.4)
		i+=1


if __name__ == "__main__":
	try:
		ser = serial.Serial(port='COM5', baudrate=115200, timeout=3)
	except OSError as err:
		print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
		# print('\n \t Retrying in 5s')
		sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

	print("**Connected to: " + ser.portstr +'**')
	
	f = open('Measurements/SPHERE_IMU_Measurements_NormalVSComplementary.txt', 'w+')
	
	start = time.time()
	
	data_acquisition()
	
	end = time.time()
	print(end-start)
	
	ser.close()