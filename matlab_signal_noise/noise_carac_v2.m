%% DECODING SECTION
clear all; close all;

% opens the JSON file decodes it
fname = 'json_backup/20190312am_noise_carac_v2.json';
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
histfit(-ESP); grid on;
xlabel('Signal [dBm]');
ylabel('Occurences [-]');
title('Histogram fit for signal');


%% STAT SECTION
[H, p] = jbtest(ESP);       % result: does not follow it...
% [H2, p2, ksstat, cv] = kstest(ESP, 'Alpha', 0.1);