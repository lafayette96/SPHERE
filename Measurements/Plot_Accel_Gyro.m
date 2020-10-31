
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data
IMU = load('SPHERE_IMU_Measurements_Gyro_accel.txt');
[m,n] = size(IMU); % get size

Accelerometer = IMU(:,1:3);
Gyroscope = IMU(:,4:6);
Magnetometer = load('Magnetometer.txt');
time = linspace(0,10,m);

%% Plot data
figure('Name', 'Sensor Data');
axis(1) = subplot(2,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(2,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
% axis(3) = subplot(3,1,3);
% hold on;
% plot(time, Magnetometer(:,1), 'r');
% plot(time, Magnetometer(:,2), 'g');
% plot(time, Magnetometer(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Flux (G)');
% title('Magnetometer');
% hold off;
linkaxes(axis, 'x');