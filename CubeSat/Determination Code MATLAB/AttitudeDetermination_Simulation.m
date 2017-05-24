%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AEM 4331
% Attitude Determination Simulation
%
% Patipan Pipatpinyopong 11/5/16
% 
% Modified:
% 11/30/16 - switch importSTK to import_STK function which includes angular
% velocity data. Update sim_time variable to 10Hz rate. Add W_in to run
% simulation. Add plot to compare both TRIAD and EKF quaternion results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
d2r = pi/180;
r2d = 1/d2r;

% Load results from STK
[Day, Month, Year, Hour, Min, Sec, Latdeg, Londeg, Altkm, Sun, M_nT, ...
 W, q_STK] ...
        = import_STK('Cubesat AttitudeDetermination_Short.csv');

% Set simulation time (1 minute increment)
sim_time = 0:0.1:(length(Day)-1)*0.1;
% Create UTC time vector [year month day hr min sec]
UTC = timeseries([Year Month Day Hour Min Sec], sim_time);

% Create timeseries data from sim inputs
m_B_in = timeseries(M_nT, sim_time);
s_B_in = timeseries(Sun, sim_time);
Alt_m = timeseries(Altkm.*1000, sim_time);
Lat = timeseries(Latdeg, sim_time);
Lon = timeseries(Londeg, sim_time);
W_in = timeseries(W*d2r, sim_time);
% Add bias to gyros
% W_in = timeseries(W*d2r+[0.001 0.002 -0.0015], sim_time); 
%% Run the simulation -> Select the model to run

sim('Attitude_Determination.slx', [0 sim_time(end)])

%% Compare algorithm output to STK data
for i = 1:length(q_STK(:,1)) % Make sure scalar part is positive
    if q_STK(i,1) < 0
        q_STK(i, :) = -q_STK(i, :);
    end
end

q_err = quatmultiply(q, quatconj(q_STK));% [Nx4] array of quaternion errors
% make sure scalar is less than one. Precision caused some values to go
% slightly above 1 and acos will output complex number
for j = 1:length(q_err(:,1)) 
    if q_err(j,1) > 1
        q_err(j,1) = 1;
    end
    if q_err(j,1) < 0
        q_err(j,1) = -q_err(j,1);
    end
end
att_err = wrapTo180(2*acos(q_err(:,1))*r2d); % [deg] angle difference. 

%% Plot quaternions EKF
figure(1)
subplot(4,1,1);
hold on; grid on;
plot(sim_time, q_STK(:, 1), '-.r', 'LineWidth', 2);
plot(sim_time, q(:,1), '--k', 'LineWidth', 2);
% xlim([0 200])
ylabel('q_0 (scalar)')
xlabel('Time (sec)')
legend('STK data', 'Sim Results');
set(gca, 'FontSize', 14);
hold off;

subplot(4,1,2);
hold on; grid on;
plot(sim_time, q_STK(:, 2), '-.r', 'LineWidth', 2);
plot(sim_time, q(:,2), '--k', 'LineWidth', 2);
% xlim([0 200])
ylabel('q_1')
xlabel('Time (sec)')
set(gca, 'FontSize', 14);
hold off;

subplot(4,1,3);
hold on; grid on;
plot(sim_time, q_STK(:, 3), '-.r', 'LineWidth', 2);
plot(sim_time, q(:,3), '--k', 'LineWidth', 2);
% xlim([0 200])
ylabel('q_2')
xlabel('Time (sec)')
set(gca, 'FontSize', 14);
hold off;

subplot(4,1,4);
hold on; grid on;
plot(sim_time, q_STK(:, 4), '-.r', 'LineWidth', 2);
plot(sim_time, q(:,4), '--k', 'LineWidth', 2);
% xlim([0 200])
ylabel('q_3')
xlabel('Time (sec)')
set(gca, 'FontSize', 14);
hold off;

% %% Plot measurement update quaternion
% figure
% subplot(4,1,1);
% hold on; grid on;
% plot(sim_time, q_STK(:, 1), '-.r', 'LineWidth', 2);
% plot(sim_time, q_update(:,1), '--k', 'LineWidth', 2);
% xlim([0 200])
% ylabel('q_0 (scalar)')
% xlabel('Time (sec)')
% legend('STK data', 'Sim Results');
% set(gca, 'FontSize', 14);
% hold off;
% 
% subplot(4,1,2);
% hold on; grid on;
% plot(sim_time, q_STK(:, 2), '-.r', 'LineWidth', 2);
% plot(sim_time, q_update(:,2), '--k', 'LineWidth', 2);
% xlim([0 200])
% ylabel('q_1')
% xlabel('Time (sec)')
% set(gca, 'FontSize', 14);
% hold off;
% 
% subplot(4,1,3);
% hold on; grid on;
% plot(sim_time, q_STK(:, 3), '-.r', 'LineWidth', 2);
% plot(sim_time, q_update(:,3), '--k', 'LineWidth', 2);
% xlim([0 200])
% ylabel('q_2')
% xlabel('Time (sec)')
% set(gca, 'FontSize', 14);
% hold off;
% 
% subplot(4,1,4);
% hold on; grid on;
% plot(sim_time, q_STK(:, 4), '-.r', 'LineWidth', 2);
% plot(sim_time, q_update(:,4), '--k', 'LineWidth', 2);
% xlim([0 200])
% ylabel('q_3')
% xlabel('Time (sec)')
% set(gca, 'FontSize', 14);
% hold off; 
%% Plot attitude estimation error
figure(3);
grid on; hold on;
plot(sim_time, att_err, 'o','MarkerSize', 2);
xlabel('Time (sec)')
ylabel('Attitude Error (\circ)');
% xlim([0 200])
set(gca, 'FontSize', 14);
hold off;