close all
clear all

% Read IMU measurements
%IMU = load('SPHERE_IMU_Measurements.txt');
IMU = load('SPHERE_IMU_Measurements2.txt');

%Check the size of received array 
[m,n] = size(IMU);

%Save array columns to separate values and create time
time = IMU(:,1);
roll = IMU(:,2);
pitch = IMU(:,3);
timestamp = linspace(0,10,m);

%% Plot and scatter plot the values
figure(1)
plot(time, roll)
title('Pich and roll plot')
hold on
plot(time, pitch)
hold off
legend('Roll', 'Pitch')
ylabel('Angle (deg)') 
xlabel('Time (sec)') 

% figure(2)
% scatter(time, pitch)
% title('Pich and roll scatter plot')
% hold on
% scatter(time, roll)
% hold off

%% Moving Average
lag = 20; %can be 20 as well
roll_movavg = movmean(roll, lag);
pitch_movavg = movmean(pitch, lag);

figure(3)
plot(time, roll_movavg)
title('Moving average')
hold on
plot(time, pitch_movavg)
hold off
legend('Roll averaged', 'Pitch averaged')
ylabel('Angle (deg)') 
xlabel('Time (sec)')


figure(4)
plot(time, roll_movavg)
title('Moving average')
hold on
plot(time, pitch_movavg)
plot(time, roll)
plot(time, pitch)
hold off
grid on
legend('Roll averaged', 'Pitch averaged', 'Roll', 'Pitch')
ylabel('Angle (deg)') 
xlabel('Time (sec)')

figure(5)
plot(time, roll_movavg)
title('Moving average')
hold on
plot(time, roll)
hold off
legend('Roll averaged', 'Roll')
ylabel('Angle (deg)') 
xlabel('Time (sec)')

figure(6)
plot(time, pitch_movavg)
title('Moving average')
hold on
plot(time, pitch)
hold off
legend('Pitch averaged', 'Pitch')
ylabel('Angle (deg)') 
xlabel('Time (sec)')