%% DECODING SECTION
clear all; close all;

% opens the JSON file decodes it
fname = 'json_backup/20190516_new_antenna_v2.json';
[time, distances, SF, RSSI, ESP, SNR, means_RSSI, means_ESP, means_SNR] = decode_json(fname);


%% PLOTTING SECTION
% figure();
% plot(ESP, 'r'); grid on; hold on;
% plot(RSSI, 'b');
% xlabel('Measure number [-]');
% ylabel('Signal [dBm]');
% title('Signal received');
% legend('ESP', 'RSSI');

figure();
histfit(ESP-mean(ESP)); grid on;
xlabel('Signal [dBm]');
ylabel('Occurences [-]');
title('Distance of 50m');


% %% STAT SECTION
% mean_esp = mean(ESP);
% std_esp = std(ESP);
% test_cdf = makedist('tlocationscale','mu',mean_esp,'sigma',std_esp,'nu',1);
% [h, p] = kstest(ESP, 'CDF', test_cdf, 'Alpha', 0.01);