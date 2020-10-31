close all
clear all

% Read IMU measurements
%IMU = load('SPHERE_IMU_Measurements.txt');
IMU = load('servo_response13.txt');

%Check the size of received array 
[m,n] = size(IMU);

roll_raw = IMU(:,1)-1;
roll = IMU(:,2)-1;
control_val = IMU(:,3);
time = linspace(0,10,m);
roll_raw_movmean = movmean(roll_raw, 15);



figure(1)
plot(time, roll,'b')
hold on
%plot(time, roll_raw)
plot(time, roll_raw_movmean,'g')
plot(time, control_val,'r')
title('Servo response to different control values')
legend('Roll - Complementary', 'Roll - Raw', 'Roll - Raw w/ moving average')
ylabel('Roll Angle (degrees)') 
xlabel('Time (seconds)') 
grid on
grid minor

