close all
clear all

% Read IMU measurements
%IMU = load('SPHERE_IMU_Measurements_Complementary.txt');
IMU = load('SPHERE_IMU_Measurements_Kalman.txt');

%Check the size of received array 
[m,n] = size(IMU);

KalmanX = IMU(:,1);
KalmanY = IMU(:,2);

time = linspace(0,6.59,m);

figure(1)
plot(time, KalmanX)
title('Kalman Pitch angle')
legend('Pitch - Complementary filter', 'Pitch - Gyroscope', 'Pitch - Accelerometer')
ylabel('Angle (degrees)') 
xlabel('Time (seconds)') 
grid on


figure(2)
plot(time, KalmanY)
title('Kalman Roll angle')
legend('Pitch - Complementary filter', 'Pitch - Gyroscope', 'Pitch - Accelerometer')
ylabel('Angle (degrees)') 
xlabel('Time (seconds)') 
grid on