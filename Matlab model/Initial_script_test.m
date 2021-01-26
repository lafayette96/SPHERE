
%% Parameter initialization 
%l    = 0.029;       % length of pendulum rod
l    = 0.0255;      % length of pendulum rod
%l    = 0.029;      % length of pendulum rod

M1   = 1.38;                          % sphere 
M2   = 0.613;
%M1   = 0.907;                       % sphere 
%M2   = 0.39;                        % pendulum mass

Tv   = -0.0028;                       % coefficient of friction (COF)
%Tv   = -0.0035;                      % coefficient of friction (COF)

R    = 0.125;                        % sphere radius
%R    = 0.1235;                      % sphere radius

J1   = 1.6*(2/3)*M1*R^2;             % sphere's moment of inertia
g    = 9.81;                         % gravitational acceleration
%T    = 0.35;                        % pendulum's natural frequency (eigenfrequency)
T    = (2*pi)/sqrt(g/l);             % pendulum's natural frequency (eigenfrequency)
J2   = 0.3*M2*g*l*(T/(2*pi))^2;      % pendulum's moment of inertia

L    = -[ M2*l*(-2*l + R) - 2*J2, 0, -2*M2*l*g ];
M    =  [ J1 + 2*J2 + M1*R^2 + M2*(2*l^2 - 3*R*l + R^2), -Tv, 2*l*g*M2 ];

A1 = J1 + J2 + M1*R*R + M2*R*R + M2*l*l;
A2 = M2*R*l; 
A3 = J2 + M2*l*l;
A4 = M2 * g;

L_trans = [A2-2*A3 0 -2*A4];
M_trans = [A1 - 3*A2 + A3 -Tv 2*A4];



%Servo
K    = 25.937;
Top  = 0.015;
Smin = -480;
Smax = 502;
delay_r = 0.05;
Tprob = 0.014;

angular_position = deg2rad(45);

Kp = 0.7;
Ki = 0.1;
Kd = 0.35;

%IMU = load('SPHERE_roll_0_to_16deg.txt');
IMU = load('capture_roll_0_max.txt');
[m,n] = size(IMU);

Pitch = (IMU(:,1));
Roll = (IMU(:,2)+2);
%Roll(1:220) = Roll(1:220)*0.2;
TIM2_CCR2 = (IMU(:,3));
TIM1_CCR3 = (IMU(:,4));

Final_val = -28;
Step_time = 4.45;

time = linspace(0,35,m);


sim('SPHERE_model4.slx');

simulation_data1 = ans.position.Data;
simulation_data1 = simulation_data1*0.96;

setpoint = ans.setpoint.Data;
%simulation_data(1:700,1) = -40;
setpoint(72:1051,1) = 14;
setpoint(1051:2101,1) = -14;


%error_Fuzzy = setpoint - simulation_data;


%Calculate error
%Roll2 = Roll(1:2451,:);
error = simulation_data1(100:1500)-Roll(100:1500);

SSE = sum(error.*error)
SSE_dec = num2str(SSE,'%.5f')

MSE = mean(error.*error)

MAE = mean(error)

%13 - 912
%15s - 1053 samples
%17s - 1193

figure(1)
plot(ans.setpoint.Time(100:1500)-3.1, simulation_data1(100:1500),'LineWidth', 1.5);
hold on
plot(time(100:1000)-3.1, Roll(100:1000), 'LineWidth', 1.5)
legend('Simulated Roll Data', 'Measured Roll Data' )
grid on
grid minor
ylabel('Roll angle (degrees)');
xlabel('Time (seconds)');
xlim([0 12])
ylim([-60 10])
set(gca,'FontSize',25)

% figure(1)
% plot(ans.setpoint.Time(1:2100), simulation_data(1:2100));
% hold on
% plot(time(1:2100), Roll(1:2100))
% legend('Simulated Roll Data', 'Measured Roll Data' )
% grid on
% grid minor
% ylabel('Roll angle (degrees)');
% xlabel('Time (seconds)');

% figure(2)
% plot(ans.setpoint.Time, simulation_data);
% hold on
% plot(ans.setpoint.Time, setpoint)
% legend('Simulated Roll Angle - Fuzzy Controller', 'Setpoint' )
% %title('P=0.9, I=0.035, D=0.07')
% grid on
% ylabel('Roll angle (degrees)');
% xlabel('Time (seconds)');



% % figure(3)
% % plot(ans.simout1.Time, error_Fuzzy)
% % ylabel('Error (degrees)');
% % xlabel('Time (seconds)');
% % grid on
% % 
% % figure(4)
% % subplot(1,2,1)
% % plotmf(FUZZY_controller,'input',1)
% % title('Input 1')
% % subplot(1,2,2)
% % plotmf(FUZZY_controller,'input',2)
% % title('Input 2')
% % 
% % figure(5)
% % plotmf(FUZZY_controller,'output',1)
% % 
% % opt = gensurfOptions;
% % opt.NumGridPoints = 70;
% % 
% opt = gensurfOptions;
% %opt.InputIndex = [2 3];
% opt.NumGridPoints = 50;
% 
% figure(6)
% gensurf(fis,opt)
% title('Control surface of type-2 FIS used in spherical robot control')

% figure(2)
% plot(ans.simout.Time, error);
% grid on
% grid minor
% ylabel('Error (degrees)');
% xlabel('Time (seconds)');


% figure(2)
% plot(ans.simout.Time, simulation_data);
% hold on
% plot(time, Roll)
% legend('Simulation Roll Data', 'Measured Roll Data' )
% grid on
