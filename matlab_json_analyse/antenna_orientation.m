clear all; close all;

% get data
filename_45 = '20190215pm_antenna45.json';
filename_up = '20190215pm_antennaup.json';
filename_hor = '20190215pm_antennahor.json';
[time45, distances45, RSSI45, ESP45, means_RSSI45, means_ESP45] = decode_json(filename_45);
[timeup, distancesup, RSSIup, ESPup, means_RSSIup, means_ESPup] = decode_json(filename_up);
[timehor, distanceshor, RSSIhor, ESPhor, means_RSSIhor, means_ESPhor] = decode_json(filename_hor);

% calibration distances
d_calib = [10 20 50 100 150 200];


% plot mean ESP and RSSI against distances
figure();
plot(d_calib, means_ESP45, 'ro-'); hold on; grid on;
plot(d_calib, means_ESPup, 'bo-');
plot(d_calib, means_ESPhor, 'go-');
legend('45', 'up', 'hor');
xlabel('Distance [m]');
ylabel('ESP [dBm]');
title('Mean ESP');