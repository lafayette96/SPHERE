close all
clear all

% Read IMU measurements
%IMU = load('SPHERE_IMU_Measurements.txt');
IMU = load('SPHERE_IMU_Measurements_KalmanVSMadgwick.txt');

%Check the size of received array 
[m,n] = size(IMU);

KalmanX = IMU(:,1);
KalmanY = -IMU(:,2);
Madg_Roll = IMU(:,3);
%CompX = IMU(:,4);

time = linspace(0,7.79,m);

figure(1)
plot(time, KalmanX)
title('SPHERE - Complementary Filter vs Madgwick Filter')
hold on
plot(time, KalmanY)
plot(time, Madg_Roll)
hold off
legend('Roll - Kalman ', 'Pitch - Kalman', 'Pitch - Madgwick')
ylabel('Angle (degrees)') 
xlabel('Time (seconds)') 
grid on