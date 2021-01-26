close all

IMU1 = load('capture101.txt'); %0-35deg No controller

%Read values for no controller
Pitch1 =            (IMU1(:,1));
Roll1 =             (IMU1(:,2));
TIM2_CCR21 =        (IMU1(:,3));
TIM1_CCR31 =        (IMU1(:,4));
Controller_type1 =  (IMU1(:,5));
Setpoint1 =         (IMU1(:,6));

[m,n] = size(IMU1);

time = linspace(0,100,m);

figure(1)
plot(time, Setpoint1, 'r--', 'LineWidth', 1)
hold on
plot(time, Roll1, 'm');
legend(' Setpoint' ,' Measured Roll Data - No controller',' Measured Roll Data - PID controller', ' Measured Roll Data - Fuzzy controller', 'FontSize',12);
grid on
grid minor
ylabel('Roll angle (degrees)');
xlabel('Time (seconds)');