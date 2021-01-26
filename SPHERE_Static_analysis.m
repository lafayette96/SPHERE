%% Prints 
close all
clear all

IMU_raw = load('Measurements/SPHERE_IMU_Measurements_Servo_Angle.txt');

%IMU = sort(IMU_raw); %sort data, sine it is 'glued' with 4 different measurements
IMU = sortrows(IMU_raw.',1).';

[m,n] = size(IMU);

duty_cycle = IMU(:,1);
position = IMU(:,2)-1.51;

figure(1)
plot(duty_cycle, position, 'o')
%plot(duty_cycle, position)
%title('SPHERE - Complementary Filter vs Kalman Filter')
hold on
ylabel('Servo Angle (degrees)') 
xlabel('PWM Duty Cycle (%)') 
grid on

[c,S]= polyfit(duty_cycle,position,1)
%a=-0.5227    b=0.0610

[position_est,delta] = polyval(c,duty_cycle,S);

plot(duty_cycle,position_est);
%plot(duty_cycle,position_est+2*delta,'m--',duty_cycle,position_est-2*delta,'m--')

