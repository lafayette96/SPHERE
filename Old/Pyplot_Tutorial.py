#Pyplot tutorial

import matplotlib.pyplot as plt
import pandas as pd


#f = open('SPHERE_IMU_Measurements_Samples.txt', 'r')

df = pd.read_csv('SPHERE_IMU_Measurements_Samples.txt', header=None)

roll = df.iloc[:,0].tolist()
pitch  = df.iloc[:,1].tolist()

print(roll)
print(pitch)

time = list(range(len(roll)))

#plt.plot(time, df)
df.plot()
#plt.plot(time, roll)

plt.show()

