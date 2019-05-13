%% DECODING SECTION
clear all; close all;

% opens the JSON file decodes it
fname = 'json_backup/20190308pm_torus_hor_indoor_v8.json';
[time, angles, SF, RSSI, ESP, SNR, ~, ~, ~, tx_pow] = decode_json(fname);

% modify because error
angles(110:end) = angles(110:end) - 1;
angles = (angles - 1) * 45;

% angles
angles_list = [0, 45, 90, 135, 180, 225, 270, 315];

% % number of messages received
% nb_0 = sum(angles==1);
% nb_45 = sum(angles==2);
% nb_90 = sum(angles==3);
% nb_135 = sum(angles==4);
% nb_180 = sum(angles==5);
% nb_225 = sum(angles==6);
% nb_270 = sum(angles==7);
% nb_315 = sum(angles==8);


%% PLOTTING SECTION

% % plot signal against time
% figure();
% plot(time, ESP, 'r'); grid on; hold on;
% plot(time, RSSI, 'b');

% plot signal against measure number
figure();
plot(ESP, 'r'); grid on; hold on;
plot(RSSI, 'b');
legend('ESP', 'RSSI');
title('Measured signal strength');
xlabel('Measure number [-]');
ylabel('Signal [dBm]');

% % boxplot
% figure();
% boxplot(ESP, 'Width', 10); grid on;
% title('Measured signal strength');
% ylabel('Signal [dBm]');

% histogram
figure();
histfit(ESP); grid on;
title('Measured signal strength');
xlabel('Signal [dBm]');
ylabel('Occurences [-]');
