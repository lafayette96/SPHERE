close all
clear all

% Read IMU measurements
%IMU = load('SPHERE_IMU_Measurements.txt');
IMU = load('KalmanVComp2.txt');

%Check the size of received array 
[m,n] = size(IMU);

roll = IMU(:,1)+0.35;
pitch = IMU(:,2);
KalmanX = IMU(:,3);
KalmanY = -IMU(:,4);
magd_roll = IMU(:,5);
magd_pitch = IMU(:,6);
TIM2 = -IMU(:,7)+ 58;

error = KalmanX - pitch; 

time = linspace(0,20,m);

figure(1)
plot(time, KalmanX)
title('SPHERE - Complementary Filter vs Kalman Filter')
hold on
plot(time, pitch)
%plot(time, TIM2)
hold off
legend('Roll - Kalman ', 'Roll - Complementary')
ylabel('Angle (degrees)') 
xlabel('Time (seconds)') 
grid on


figure(2)
plot(time, KalmanY)
%title('SPHERE Complementary Filter vs Kalman Filter')
hold on
plot(time, roll)
hold off
legend('Pitch - Kalman', 'Pitch - Complementary')
ylabel('Angle (degrees)') 
xlabel('Time (seconds)') 
grid on

figure(3)
plot(time, KalmanY)
title('SPHERE Complementary Filter vs Kalman Filter')
hold on
plot(time, magd_roll)
plot(time, magd_pitch)
plot(time, roll)
hold off
legend('Pitch - Kalman', 'Roll - Magd', 'Pitch  - Magd', 'Pitch - Comp')
ylabel('Angle (degrees)') 
xlabel('Time (seconds)') 
grid on

figure(4)
plot(time, error)
%title('SPHERE Complementary Filter vs Kalman Filter')
ylabel('Error (degrees)') 
xlabel('Time (seconds)') 
grid on