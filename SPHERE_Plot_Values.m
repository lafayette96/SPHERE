close all
clear all

% Read IMU measurements
%IMU = load('SPHERE_IMU_Measurements.txt');
IMU = load('Measurements/SPHERE_IMU_Measurements_NormalVSComplementary.txt');

%Check the size of received array 
[m,n] = size(IMU);

%Save array columns to separate values and create time
%timestamp = IMU(:,1);
pitch_comp = (IMU(:,1));
pitchGyro = (IMU(:,2)-1);
pitchAccel = (IMU(:,3));


time = linspace(0,4.5,m);

%200 samples - 4.5s
%Sampling time = 45HZ 

%% Plot and scatter plot the values

figure(1)
plot(time, pitch_comp)
title('SPHERE Pitch angle')
hold on
plot(time, pitchGyro)
plot(time, pitchAccel)
hold off
legend('Pitch - Complementary filter', 'Pitch - Gyroscope', 'Pitch - Accelerometer')
ylabel('Angle (degrees)') 
xlabel('Time (seconds)') 
grid on





% figure(12)
% plot(time, pitch_comp)
% title('SPHERE Pitch angle Different Params')
% hold on
% plot(time, pitch_comp2)
% plot(time, pitchGyro)
% plot(time, pitchAccel)
% hold off
% legend('Pitch - Complementary filter 0.98-0.02','Pitch - Complementary filter 0.99-0.01', 'Pitch - Gyroscope', 'Pitch - Accelerometer')
% ylabel('Angle (degrees)') 
% xlabel('Time (seconds)') 
% grid on


% gx = IMU(:,5);
% gy = IMU(:,6);
% gz = IMU(:,7);
% roll = IMU(:,8);
% pitch = IMU(:,9);

% time = IMU(:,1);
% roll = (IMU(:,2));
% pitch = -(IMU(:,3));
% pitch_comp = (IMU(:,4));
% roll_comp = IMU(:,5);


% figure(1)
% plot(time, pitch_comp)
% title('Pich and roll plot')
% hold on
% plot(time, pitchGyro)
% plot(time, pitchAccel)
% hold off
% legend('Roll', 'Pitch')
% ylabel('Angle (degrees)') 
% xlabel('Time (seconds)') 

%% Moving Average
%lag = 20; %can be 20 as well
%roll_movavg = movmean(roll, lag);
%pitch_movavg = movmean(pitch_comp, lag);


% figure(2)
% plot(time, roll_comp)
% title('Pich and roll plot')
% hold on
% plot(time, pitch_comp)
% hold off
% legend('Roll', 'Pitch')
% ylabel('Angle (deg)') 
% xlabel('Time (sec)')

% figure(2)
% scatter(time, pitch)
% title('Pich and roll scatter plot')
% hold on
% scatter(time, roll)
% hold off

% %% Moving Average
% lag = 20; %can be 20 as well
% roll_movavg = movmean(roll, lag);
% pitch_movavg = movmean(pitch, lag);
% 
% figure(3)
% plot(time, roll_movavg)
% title('Moving average')
% hold on
% plot(time, pitch_movavg)
% hold off
% legend('Roll averaged', 'Pitch averaged')
% ylabel('Angle (deg)') 
% xlabel('Time (sec)')
% 
% 
% figure(4)
% plot(time, roll_movavg)
% title('Moving average')
% hold on
% plot(time, pitch_movavg)
% plot(time, roll)
% plot(time, pitch)
% hold off
% grid on
% legend('Roll averaged', 'Pitch averaged', 'Roll', 'Pitch')
% ylabel('Angle (deg)') 
% xlabel('Time (sec)')
% 
% figure(5)
% plot(time, roll_movavg)
% title('Moving average')
% hold on
% plot(time, roll)
% hold off
% legend('Roll averaged', 'Roll')
% ylabel('Angle (deg)') 
% xlabel('Time (sec)')
% 
% figure(6)
% plot(time, pitch_movavg)
% title('Moving average')
% hold on
% plot(time, pitch)
% hold off
% legend('Pitch averaged', 'Pitch')
% ylabel('Angle (deg)') 
% xlabel('Time (sec)')