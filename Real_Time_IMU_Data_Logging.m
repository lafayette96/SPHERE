clear all
clc

%% User Defined Properties
serialPort = 'COM5';            % define COM port #
plotTitle = 'Serial Data Log - IMU Roll and Pitch ';  % plot title
xLabel = 'Elapsed Time (s)';    % x-axis label
yLabel = 'Angle (deg)';         % y-axis label
plotGrid = 'on';                % 'off' to turn off grid
min = -110;                     % set y-min
max = 110;                      % set y-max
scrollWidth = 50;               % display period in plot, plot entire data log if <= 0
delay = .001;                    % make sure sample faster than resolution
%Define Function Variables
time = 0;
data_x = 0;
data_y = 0;
count = 0;

%% Set up Plot
plotGraphx = plot(time,data_x,'-mo',...
    'LineWidth',0.5,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[.49 1 .63],...
    'MarkerSize',2);
hold on;  % it will hold all the lines on same axis
plotGraphy = plot(time,data_y,'-mo',...   <---- define data_y before this
    'LineWidth',0.5,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[.49 1 .63],...
    'MarkerSize',2);
title(plotTitle,'FontSize',25);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
axis([0 10 min max]);
grid(plotGrid);

%% Open Serial COM Port
s = serial(serialPort);
disp('Close Plot to End Session');
fopen(s);
tic

while ( ishandle(plotGraphx)  ) %Loop when Plot is Active && ishandle(plotGraphy) && ishandle(plotGraphz)
    dat = fscanf(s,['%f,%f \n']); %Read Data from Serial as Float: PLEASE NOTE: Here i modified the code, in order to adjust the correct format of data in input.
    if(~isempty(dat) && isfloat(dat)) %Make sure Data Type is Correct
        count = count + 1;
        time(count) = toc;    %Extract Elapsed Time
        data_x(count) = dat(1);
        data_y(count) = dat(2);
        %Set Axis according to Scroll Width
        if(scrollWidth > 0)
            set(plotGraphx,'XData',time(time > time(count)-scrollWidth),'YData',data_x(time > time(count)-scrollWidth));
            set(plotGraphy,'XData',time(time > time(count)-scrollWidth),'YData',data_y(time > time(count)-scrollWidth));
            axis([time(count)-scrollWidth time(count) min max]);
        else
            set(plotGraphx,'XData',time,'YData',data_x);
            set(plotGraphy,'XData',time,'YData',data_y);
            axis([0 time(count) min max]);
        end
        %Allow MATLAB to Update Plot
        pause(delay);
    end
end
%Close Serial COM Port and Delete useless Variables
fclose(s);
clear count dat delay max min plotGraph plotGrid plotTitle s ...
    scrollWidth serialPort xLabel yLabel;
disp('Session Terminated...');