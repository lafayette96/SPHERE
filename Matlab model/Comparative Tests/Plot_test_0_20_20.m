close all

IMU1 = load('capture81.txt'); %0-35deg No controller
IMU2 = load('capture88.txt'); %0-35deg PID
IMU3 = load('capture64.txt'); %0-35deg Fuzzy


%Read values for no controller
Pitch1 =            (IMU1(1:1398,1));
Roll1 =             (IMU1(1:1398,2))+0.2;
TIM2_CCR21 =        (IMU1(1:1398,3));
TIM1_CCR31 =        (IMU1(1:1398,4));
Controller_type1 =  (IMU1(1:1398,5));
Setpoint1 =         (IMU1(1:1398,6));

%Read values for PID
Pitch2 =            (IMU2(1:1398,1));
Roll2 =             (IMU2(1:1398,2)+4.5);
TIM2_CCR22 =        (IMU2(1:1398,3));
TIM1_CCR32 =        (IMU2(1:1398,4));
Controller_type2 =  (IMU2(1:1398,5));
Setpoint2 =         (IMU2(1:1398,6));

%Read values for Fuzzy
Pitch3 =            (IMU3(1:1398,1));
Roll3 =             (IMU3(1:1398,2));
TIM2_CCR23 =        (IMU3(1:1398,3));
TIM1_CCR33 =        (IMU3(1:1398,4));
Controller_type3 =  (IMU3(1:1398,5));
Setpoint3 =         (IMU3(1:1398,6));



Setpoint1(1:222) = -40;
% Setpoint3(210:1398) = 10;

% Roll1 = Roll1*1,17 + 6;
% Roll2 = Roll2*1.17 + 6;
Roll3 = Roll3*1.17 + 6;


% Roll1(222:1398) = Roll1(222:1398)+ 5;
% Roll2(222:1398) = Roll2(222:1398)+ 5;
% Roll3(222:1398) = Roll3(222:1398)+ 5;


%set time
[m,n] = size(IMU1);
m = 1398;
time = linspace(0,20,m);

figure(1)
plot(time, Setpoint1, 'r--', 'LineWidth', 1)
hold on
plot(time, Roll1, 'm');
plot(time, Roll2, 'g', 'LineWidth', 1.5);
plot(time, Roll3, 'b', 'LineWidth', 1.5);
legend(' Setpoint' ,' Measured Roll Data - No controller',' Measured Roll Data - PID controller', ' Measured Roll Data - Fuzzy controller', 'FontSize',12);
grid on
grid minor
ylabel('Roll angle (degrees)');
xlabel('Time (seconds)');


%plot roll
% figure(2)
% plot(time, Setpoint1, 'r--', 'LineWidth', 1)
% hold on
% plot(time, Roll1, 'm');
% plot(time, Roll2, 'g', 'LineWidth', 1.5);
% plot(time, Roll3, 'b', 'LineWidth', 1.5);
% legend(' Setpoint' ,' Measured Roll Data - No controller',' Measured Roll Data - PID controller', ' Measured Roll Data - Fuzzy controller', 'FontSize',12);
% grid on
% grid minor
% ylabel('Roll angle (degrees)');
% xlabel('Time (seconds)');


%% Calculate quality measures 

%Calculate error
error1 = Setpoint1-Roll1;
error2 = Setpoint1-Roll2;
error3 = Setpoint1-Roll3;

% error1(1:209) = error11(1:209)-1.6;
% error2(1:209) = error22(1:209)-1.6;
% error3(1:209) = error33(1:209)-1.6;

%plot error
figure(2)
plot(time, error1, 'm--', 'LineWidth', 1);
hold on
plot(time, error2, 'g', 'LineWidth', 1.5);
plot(time, error3, 'b', 'LineWidth', 1.5);
legend(' Error - no controller' ,' Error - PID',' Error - Fuzzy','FontSize',12)
grid on
grid minor
ylabel('Error (degrees)');
xlabel('Time (seconds)');



%Mean Absolute Error
MAE1 = sum(error1)
MAE2 = sum(error2)
MAE3 = sum(error3)

%Squared Sum Error
SSE1 = sum(error1.*error1);
SSE1_dec = num2str(SSE1,'%.2f')

SSE2 = sum(error2.*error2);
SSE2_dec = num2str(SSE2,'%.2f')

SSE3 = sum(error3.*error3);
SSE3_dec = num2str(SSE3,'%.2f')

%Mean Square Error
MSE1 = mean(error1.*error1)
MSE2 = mean(error2.*error2)
MSE3 = mean(error3.*error3)

