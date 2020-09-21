close all
clear all

% Read IMU measurements
IMU = load('SPHERE_IMU_Measurements_Gyro_Accel.txt');

%Check the size of received array 
[m,n] = size(IMU);
sampling_time = 10; %[s]

%Save array columns to separate values and create time
%time = IMU(:,1);
ax = IMU(:,2);
ay = IMU(:,3);
az = IMU(:,4);
gx = IMU(:,5);
gy = IMU(:,6);
gz = IMU(:,7);
%ay = IMU(:,2);
%az = IMU(:,3);
%gx = IMU(:,4);

accelerometerReadings = [ax, ay, az];
gyroscopeReadings = [gx, gy, gz];

%timestamp = linspace(0,sampling_time,m);

Fs = 200/8;
decim = 2;
fuse = imufilter('SampleRate',Fs,'DecimationFactor',decim);

q = fuse(accelerometerReadings,gyroscopeReadings);

time = (0:decim:size(accelerometerReadings,1)-1)/Fs;

plot(time,eulerd(q,'ZYX','frame'))
title('Orientation Estimate')
legend('Z-axis', 'Y-axis', 'X-axis')
xlabel('Time (s)')
ylabel('Rotation (degrees)')

% %% One axis plot
% accel = atan2(ay, az)*(180.0/3.1415);
% gyro = gx;
% 
% %% Moving Average
% lag = 5; %can be 20 as well
% accel_movavg = movmean(accel, lag);
% gyro_movavg = movmean(gyro, lag);
% 
% figure(1)
% plot(time, accel_movavg)
% title('Gyro and Accel')
% hold on
% plot(time, gyro_movavg)
% hold off
% legend('accelerometer', 'gyroscope')




% %% Complementary filter
% acc_angle = atan2(ay, az)*(180.0/3.1415);
% dt = sampling_time/m; %0.05
% a= 0.98;
% angle = (0);
% tau = (0.98*dt)/(1-a);
% 
% for i = 2:m-1
%     angle(i) = a * (angle(i-1) + gx(i) * dt) + ((1-a) * acc_angle(i));
% end
% 
% angle = [angle 2];
% 
% angle2 = angle -2;

%% Plots
% figure(1)
% plot(time, ax)
% title('ax and roll gx')
% hold on
% plot(time, gx)
% hold off
% legend('ax', 'gx')
% 
% figure(2)
% plot(time, ay)
% title('ay and roll gy')
% hold on
% plot(time, gy)
% hold off
% legend('ay', 'gy')
% 
% figure(3)
% plot(time, az)
% title('az and roll gz')
% hold on
% plot(time, gz)
% hold off
% legend('az', 'gz')

% figure(4)
% subplot(3,1,1)
% plot(time, ax)
% ylabel('Acc_X (deg)') 
% xlabel('Time (sec)') 
% subplot(3,1,2)
% plot(time, gx)
% ylabel('Gyro_X (deg)') 
% xlabel('Time (sec)')
% subplot(3,1,3)
% plot(time, angle2)
% ylabel('Filtered angle (deg)') 
% xlabel('Time (sec)')