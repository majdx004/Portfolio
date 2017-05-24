%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AEM 4331
% Control Values
%
% Evan Majd
% 12/2/16
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
clc 
clear vars
 
% Simulation Time
Tf = 1000;
 
% Earth Magnetic field
earth_mag = [1.9547e-05 -1.3174e-06 5.8042e-06]; % [x y z] T
 
% Magnetorquer Dipole Magnetic Moment
magnetorquer = [.24 .24 .13]; % [x y z] N*m/T
 
% Reference Commands
r = [.17 .17 .17]; % (theta) radians 
w_max = [.15/180*pi .15/180*pi .15/180*pi]; % (theta dot) rad/s
 
% Moments (Ixx, Iyy, Izz)
I = [109242619.73177/1000^3 109302085.08329/1000^3 3557528.64834/1000^3]; % kg*m^2
 
% Controller Values
damping_ratio = [1 1 1];
wn = [.01 .01 .07]; % Natural Frequency (Hz)
 
Kp = wn.^2.*I; % Proportional Constant
Kd = 2.*damping_ratio.*wn.*I; % Derivative Constant
 
% Max Torque Limitations
t_limit = abs(cross(magnetorquer,earth_mag)); % T = u x B 
 
% Poles calculation
a = [1 1 1];
b = Kd./I;
c = Kp./I;
 
p = [a(1) b(1) c(1); a(2) b(2) c(2); a(3) b(3) c(3)];
 
poles = [real(roots(p(1,:))),real(roots(p(2,:))),real(roots(p(3,:)))];
 
%% X Response
 
figure(2);
sim('matrixalgorithm',[0 Tf]);
subplot(2,2,1) 
plot(y.Time(:,1),y.Data(:,1).*(180/pi), 'r');
title('Attitude Response');
xlabel('Time (s)');
ylabel('Theta (degrees)');
grid on;
       
subplot(2,2,2)
plot(y_dot.Time(:,1),y_dot.Data(:,1).*(180/pi), 'r');
title('Angular Velocity');
xlabel('Time (s)');
ylabel('Theta Dot (degrees/s)');
grid on;
 
subplot(2,2,3)
plot(y_dot.Time(:,1),u_corrected.Data(:,1), 'r');
title('Torque Commanded');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;
hold on
plot(y_dot.Time(:,1),t_limit(1).*ones(length(y_dot.Time(:,1)),1),'k--')
hold on
plot(y_dot.Time(:,1),-t_limit(1).*ones(length(y_dot.Time(:,1)),1),'k--')
 
subplot(2,2,4)
plot([poles(1,1) poles(2,1)],[0 0], '*');
title('Poles');
xlabel('Real');
ylabel('Imaginary');
grid on;
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
axis([-1 1 -1 1])
 
% Find Steady State
temp_x = y.Data(1,1);
for i=2:(length(y.Data(:,1))-11)
    bool = abs((y.Data(i,1)-y.Data(i+10,1)));
    if bool < (1e-2)
        temp_x = y.Data(i,1);
    end
end
 
steady_state_x = temp_x*180/pi;
 
% Obtain Data
s_x = stepinfo(y.Data(:,1), y.Time,temp_x,'RiseTimeLimits',[0.05,0.95]);
 
%% Plot Graphs Y
figure(3);
subplot(2,2,1)
plot(y.Time,y.Data(:,2).*(180/pi), 'r');
title('Attitude Response');
xlabel('Time (s)');
ylabel('Theta (degrees)');
grid on;
        
subplot(2,2,2)
plot(y_dot.Time,y_dot.Data(:,2).*(180/pi), 'r');
title('Angular Velocity');
xlabel('Time (s)');
ylabel('Theta Dot (degrees/s)');
grid on;
 
subplot(2,2,3)
plot(y_dot.Time,u_corrected.Data(:,2), 'r');
title('Torque Commanded');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;
hold on
plot(y_dot.Time(:,1),t_limit(2).*ones(length(y_dot.Time(:,1)),1),'k--')
hold on
plot(y_dot.Time(:,1),-t_limit(2).*ones(length(y_dot.Time(:,1)),1),'k--')
 
subplot(2,2,4)
plot([poles(1,2) poles(2,2)],[0 0], '*');
title('Poles');
xlabel('Real');
ylabel('Imaginary');
grid on;
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
axis([-1 1 -1 1])
 
% Find Steady State
temp_y = y.Data(1,2);
for i=2:(length(y.Data(:,2))-11)
    bool = abs((y.Data(i,2)-y.Data(i+10,2)));
    if bool < (1e-2)
        temp_y = y.Data(i,2);
    end
end
 
steady_state_y = temp_y*180/pi;
 
% Obtain Data
s_y = stepinfo(y.Data(:,2), y.Time,temp_y,'RiseTimeLimits',[0.05,0.95]);
 
%% Plot Graphs Z
figure(4);
subplot(2,2,1)
plot(y.Time,y.Data(:,3).*(180/pi), 'r');
title('Attitude Response');
xlabel('Time (s)');
ylabel('Theta (degrees)');
grid on;
        
subplot(2,2,2)
plot(y_dot.Time,y_dot.Data(:,3).*(180/pi), 'r');
title('Angular Velocity');
xlabel('Time (s)');
ylabel('Theta Dot (degrees/s)');
grid on;
 
subplot(2,2,3)
plot(y_dot.Time,u_corrected.Data(:,3), 'r');
title('Torque Commanded');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;
hold on
plot(y_dot.Time(:,1),t_limit(3).*ones(length(y_dot.Time(:,1)),1),'k--')
hold on
plot(y_dot.Time(:,1),-t_limit(3).*ones(length(y_dot.Time(:,1)),1),'k--')
 
subplot(2,2,4)
plot([poles(1,3) poles(2,3)],[0 0], '*');
title('Poles');
xlabel('Real');
ylabel('Imaginary');
grid on;
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
axis([-1 1 -1 1])
 
% Find Steady State
temp_z = y.Data(1,3);
for i=2:(length(y.Data(:,3))-11)
    bool = abs((y.Data(i,3)-y.Data(i+10,3)));
    if bool < (1e-2)
        temp_z = y.Data(i,3);
    end
end
 
steady_state_z = temp_z*180/pi;
 
% Obtain Data
s_z = stepinfo(y.Data(:,3), y.Time,temp_z,'RiseTimeLimits',[0.05,0.95]);