close all
clear all

%Wczytywanie danych z pliku
data = importdata('SPHERE_IMU_Measurements_ay_az_gx.txt');
%datatmp = importdata('data2.txt');
%data = datatmp.data;
gyro_data = data(:,1);
acc_data = data(:,2:3);
 
%Skalowanie danych
%gyro_data = gyro_data*250/32768;
%acc_data = acc_data*4/65535;
 
%Wektor czasu
dt = 0.1;
%t = dt:dt:size(data, 1)*dt;
t = linspace(0,8.79,size(data, 1));
 
%Model w przestrzeni stanu
A = [1 -dt; 0 1];
B = [dt; 0];
C = [1 0];
 
%Szum pomiarowy i procesowy
std_dev_v = 1;
std_dev_w = 2;
V = [std_dev_v*std_dev_v*dt 0; 0 std_dev_v*std_dev_v*dt];
W = std_dev_w*std_dev_w;
 
% Wartosci poczatkowe
x0 = [0; 0];
P0 = [1 0; 0 1];
xpri = x0;
Ppri = P0;
xpost = x0;
Ppost = P0;
 
% Wektory wynikow
Y = zeros(1, size(t,2));
Yf = Y;
 
for i = 1:size(data, 1);
    %Obliczenie aktualnego kata na podstawie danych z akcelerometru
    acc_angle = atan(acc_data(i,1)./acc_data(i,2))*180/pi;
    u = gyro_data(i,1);
    Y(i) = acc_angle;
    
    if i == 1
        % Pierwszy pomiar sluzy nam jako wartosc poczatkowa dla filtru
        xpost = [acc_angle; 0];
    else
        % aktualizacja czasu
        xpri = A*xpost + B*u;
        Ppri = A*Ppost*A' + V;
        
        % aktualizacja pomiarow
        eps = Y(i) - C*xpri;
        S = C*Ppri*C' + W;
        K = Ppri*C'*S^(-1);
        xpost = xpri + K*eps;
        Ppost = Ppri - K*S*K';
    end
    
    %Zapis estymaty do wektora wynikow
	Yf(i) = xpost(1);
end
 
plot(t, Y);
hold on
plot(t, Yf);
title('Kalman filter')
xlabel('Time [s]')
ylabel('Accelerometer measured pitch [deg]')
legend('Measured pitch value', 'Estimated pitch value')