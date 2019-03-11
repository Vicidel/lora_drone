%% DECODING SECTION
clear all; close all;

% opens the JSON file decodes it
fname = 'json_backup/20190311pm_torus_vert_v2.json';
[time, angles, SF, RSSI, ESP, SNR, ~, ~, ~, tx_pow] = decode_json(fname);

% angle vector
angles = (angles - 1) * 15;                   % 15 for vertical, 45 for horizontal


%% PLOTTING SECTION

% plot signal against measure number
figure();
plot(ESP, 'r'); grid on; hold on;
plot(RSSI, 'b');
ESP_smooth = smooth(ESP, 5);
plot(ESP_smooth, 'm');
RSSI_smooth = smooth(RSSI, 5);
plot(RSSI_smooth, 'c');
legend('ESP', 'RSSI', 'ESP smoothed', 'RSSI smoothed');
title('Measured signal strength');
xlabel('Measure number [-]');
ylabel('Signal [dBm]');

% plot angle against time
figure();
plot(angles); grid on;
title('Angle sent');
xlabel('Measure number [-]');
ylabel('Angle [deg]');
