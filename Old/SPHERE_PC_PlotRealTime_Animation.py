import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import sys

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

# Initialize communication with IMU sensor
try:
    ser = serial.Serial(port='COM5', baudrate=9600, timeout=3)
except OSError as err:
    print("\n\t ***Cannot connect to device!*** \n\n \t {0}".format(err))
    # print('\n \t Retrying in 5s')
    sys.exit('\n\t ***Exiting***') # Terminate program imidiatelly if no device found

print("**Connected to: " + ser.portstr +'**')


# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    # Read IMU values
    received_text = ser.readline() #read line of serial input
    decoded_text = str(received_text.decode("utf-8")) # decode from bytes to utf-8
    decoded_text = decoded_text.rstrip() # remove trailing newline '\n'
    text_as_list = decoded_text.split(',') #split by the comma
    roll = text_as_list[0]
    print(roll)
	#data = np.append(data, text_as_list[0]) #append dataframe with roll value

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(roll)

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)
    #plt.axis([-100, 100, 0, 10000])

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('IMU value over Time')
    plt.ylabel('Roll (deg)')

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=10)
plt.show()
