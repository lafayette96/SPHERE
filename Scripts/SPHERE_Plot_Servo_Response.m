close all
clear all

% Read IMU measurements
%IMU = load('SPHERE_IMU_Measurements.txt');
IMU = load('Measurements/servo_response13.txt');

%Check the size of received array 
[m,n] = size(IMU);

roll_raw = IMU(:,1)-1;
roll = IMU(:,2)-1;
control_val = IMU(:,3);
time = linspace(0,10,m);
roll_raw_movmean = movmean(roll_raw, 15);
roll_movmean = movmean(roll, 3);



figure(1)
plot(time, control_val,'r', 'LineWidth', 1) %print setpoint
hold on
plot(time, roll_raw,'g', 'LineWidth', 0.5) %plot raw readings
%plot(time, roll_movmean,'b', 'LineWidth', 1)
plot(time, roll,'b', 'LineWidth', 1)
%plot(time, roll_raw_movmean,'g')
%title('Servo response to different control values')
legend('Setpoint', 'Roll - Raw readings', 'Roll - Complementary filter', 'FontSize',14)
ylabel('Roll Angle (degrees)','FontSize',16) 
xlabel('Time (seconds)', 'FontSize',16) 
grid on
grid minor

