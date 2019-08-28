clc
% close all
if ~isempty(instrfind)
    fclose(instrfind);
end

s = serial('COM11','BaudRate',512000);

fopen(s);



simT = 10; % sec
dt = 0.004;
T = 0:dt:simT;
length(T)

%%

figure('units','normalized','outerposition',[0 0 1 1]);
%%
% sub1 = subplot(2,1,1);
subplot(2,1,1);
grid on;
axis auto;
% sub1.XLim = [0 simT];
% sub1.YLim = [-90 90];
plot1_1=animatedline('Color','r','LineWidth',1);
plot1_2=animatedline('Color','b','LineWidth',1);
plot1_3=animatedline('Color','k','LineWidth',1);
%%
% sub1 = subplot(2,1,2);
subplot(2,1,2);
grid on;
axis auto;
% sub1.XLim = [0 simT];
% sub1.YLim = [-50 1050];
plot2_1=animatedline('Color','b','LineWidth',1);



%%
disp(fix(simT/dt));


%%
tic
for i = 1:fix(simT/dt)
    
%     tic
%     readasync(s);
    data = fscanf(s);
    C = textscan(data, '%f%f%f%f%f','delimiter','\t'); 
    [millis, yaw, pitch, roll, pot] = deal(C{:});
    addpoints(plot1_1, millis, yaw);
    addpoints(plot1_2, millis, pitch);
    addpoints(plot1_3, millis, roll);
    addpoints(plot2_1, millis, pot);
%     toc
    
%     sub1.XLim = [x_start+x_dx*i x_end+x_dx*i];
%     sub2.XLim = [x_start+x_dx*i x_end+x_dx*i];
    drawnow limitrate nocallbacks
 
end
toc

[yaw_x, yaw_y] = getpoints(plot1_1);
[pitch_x,pitch_y] = getpoints(plot1_2);
[roll_x, roll_y] = getpoints(plot1_3);

%%
% filter window
fw = 20;
% filter mode
fm = 'rlowess';
%%
figure('units','normalized','outerposition',[0 0 1 1]);
grid on;
%% yaw + filtered yaw speed
sub1 = subplot(3,2,1);
ylabel('yaw, yaw dt');
grid on;
hold on;
plot(yaw_x, yaw_y.*(pi/180));
% yaw_dy = diff(yaw_y.*(pi/180))./dt;
yaw_dx = diff(yaw_x) ./ 1000;
yaw_dy = diff(yaw_y.*(pi/180)) ./ yaw_dx;
yaw_dy(length(T)-1) = yaw_dy(length(T) - 2);

yaw_dy_dy = diff(yaw_dy);
yaw_dy_dy(length(T)-1) = yaw_dy_dy(length(T) - 2);

plot( yaw_x, smoothdata(yaw_dy,fm,fw) );
plot( yaw_x, smoothdata(yaw_dy_dy,fm,fw) );

%% pitch + filtered pitch speed
sub2 = subplot(3,2,3);
ylabel('pitch, pitch dt');
grid on;
hold on;
plot(pitch_x, pitch_y.*(pi/180));
% pitch_dy = diff(pitch_y.*(pi/180))./dt;
pitch_dx = diff(pitch_x) ./ 1000;
pitch_dy = diff(pitch_y.*(pi/180))./pitch_dx;
pitch_dy(length(T)-1) = pitch_dy(length(T) - 2);

pitch_dy_dy = diff(pitch_dy) ;
pitch_dy_dy(length(T)-1) = pitch_dy_dy(length(T) - 2);

plot( pitch_x, smoothdata(pitch_dy,fm,fw) );
plot( pitch_x, smoothdata(pitch_dy_dy,fm,fw) );

%% roll + filtered roll speed
sub3 = subplot(3,2,5);
ylabel('roll, roll dt');
grid on;
hold on;
plot(roll_x, roll_y.*(pi/180));
% roll_dy = diff(roll_y.*(pi/180))./dt;
roll_dx = diff(roll_x) ./ 1000;
roll_dy = diff(roll_y.*(pi/180))./roll_dx;
roll_dy(length(T)-1) = roll_dy(length(T) - 2);

roll_dy_dy = diff(roll_dy) ;
roll_dy_dy(length(T)-1) = roll_dy_dy(length(T) - 2);

plot( roll_x, smoothdata(roll_dy,fm,fw) );
plot( roll_x, smoothdata(roll_dy_dy,fm,fw) );

%%


%% yaw speed + filtered yaw speed
sub4 = subplot(3,2,2);
ylabel('yaw dt, filtered');
grid on;
hold on;
yaw_dy = diff(yaw_y.*(pi/180))./dt;
yaw_dy(length(T)-1) = yaw_dy(length(T) - 2);

plot( yaw_x, yaw_dy );

yaw_dy_filtered = smoothdata(yaw_dy,fm,fw);
plot( yaw_x, yaw_dy_filtered , 'LineWidth', 2);

%% pitch speed + filtered pitch speed
sub5 = subplot(3,2,4);
ylabel('pitch dt, filtered');
grid on;
hold on;
pitch_dy = diff(pitch_y.*(pi/180))./dt;
pitch_dy(length(T)-1) = pitch_dy(length(T) - 2);

plot( pitch_x, pitch_dy );

pitch_dy_filtered = smoothdata(pitch_dy,fm,fw);
plot( pitch_x, pitch_dy_filtered , 'LineWidth', 2);
%% yaw roll + filtered roll speed
sub6 = subplot(3,2,6);
ylabel('yaw dt, filtered');
grid on;
hold on;
roll_dy = diff(roll_y.*(pi/180))./dt;
roll_dy(length(T)-1) = roll_dy(length(T) - 2);

plot( roll_x, roll_dy );

roll_dy_filtered = smoothdata(roll_dy,fm,fw);
plot( roll_x, roll_dy_filtered , 'LineWidth', 2);
%%
fclose(s);
delete(s);
clear s