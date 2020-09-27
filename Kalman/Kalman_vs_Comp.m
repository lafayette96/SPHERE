close all
clear all

% Read IMU measurements
%IMU = load('SPHERE_IMU_Measurements.txt');
IMU = load('SPHERE_IMU_Measurements_KalmanVSComplementary.txt');

%Check the size of received array 
[m,n] = size(IMU);

KalmanX = IMU(:,1);
KalmanY = -IMU(:,2);
CompY = IMU(:,3);
CompX = IMU(:,4);

time = linspace(0,10.99,m);

figure(1)
plot(time, KalmanX)
title('SPHERE - Complementary Filter vs Kalman Filter')
hold on
plot(time, CompX)
hold off
legend('Roll - Kalman ', 'Roll - Complementary')
ylabel('Angle (degrees)') 
xlabel('Time (seconds)') 
grid on


figure(2)
plot(time, KalmanY)
title('SPHERE Complementary Filter vs Kalman Filter')
hold on
plot(time, CompY)
hold off
legend('Pitch - Kalman', 'Pitch - Complementary')
ylabel('Angle (degrees)') 
xlabel('Time (seconds)') 
grid on