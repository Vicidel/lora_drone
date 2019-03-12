%% DECODING SECTION
clear all; close all;

% opens the JSON file decodes it
fname = 'json_backup/20190311pm_torus_vert_v2.json';
[time, angles, SF, RSSI, ESP, SNR, ~, ~, ~, tx_pow] = decode_json(fname);

% angle vector
angles = (angles - 1) * 15;                   % 15 for vertical, 45 for horizontal

% measure vector
n = [1:length(angles)]';


%% PLOTTING SECTION

% % plot signal against measure number and smoothed
% figure();
% plot(ESP, 'r'); grid on; hold on;
% plot(RSSI, 'b');
% ESP_smooth = smooth(ESP, 5);
% plot(ESP_smooth, 'm');
% RSSI_smooth = smooth(RSSI, 5);
% plot(RSSI_smooth, 'c');
% legend('ESP', 'RSSI', 'ESP smoothed', 'RSSI smoothed');
% title('Measured signal strength');
% xlabel('Measure number [-]');
% ylabel('Signal [dBm]');

% plot signal against measure number
figure();
plot(n(angles==0), ESP(angles==0), 'ro'); grid on; hold on;
plot(n(angles==15), ESP(angles==15), 'go'); 
plot(n(angles==30), ESP(angles==30), 'bo'); 
plot(n(angles==45), ESP(angles==45), 'mo'); 
plot(n(angles==60), ESP(angles==60), 'co'); 
plot(n(angles==75), ESP(angles==75), 'yo'); 
plot(n(angles==90), ESP(angles==90), 'ko'); 
plot(n(angles==0), ones(size(n(angles==0)))*mean(ESP(angles==0)), 'r', 'LineWidth', 2);
plot(n(angles==15), ones(size(n(angles==15)))*mean(ESP(angles==15)), 'g', 'LineWidth', 2);
plot(n(angles==30), ones(size(n(angles==30)))*mean(ESP(angles==30)), 'b', 'LineWidth', 2);
plot(n(angles==45), ones(size(n(angles==45)))*mean(ESP(angles==45)), 'm', 'LineWidth', 2);
plot(n(angles==60), ones(size(n(angles==60)))*mean(ESP(angles==60)), 'c', 'LineWidth', 2);
plot(n(angles==75), ones(size(n(angles==75)))*mean(ESP(angles==75)), 'y', 'LineWidth', 2);
plot(n(angles==90), ones(size(n(angles==90)))*mean(ESP(angles==90)), 'k', 'LineWidth', 2);
legend('\alpha=0°', '\alpha=15°', '\alpha=30°', '\alpha=55°', '\alpha=60°', '\alpha=75°','\alpha=90°');
title('Measured signal strength');
xlabel('Measure number [-]');
ylabel('Signal [dBm]');

% % plot angle against time
% figure();
% plot(angles); grid on;
% title('Angle sent');
% xlabel('Measure number [-]');
% ylabel('Angle [deg]');
