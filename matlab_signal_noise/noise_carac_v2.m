%% DECODING SECTION
clear all; close all;

% opens the JSON file decodes it
fname = 'json_backup/20190311pm_noise_carac.json';
[time, distances, SF, RSSI, ESP, SNR, means_RSSI, means_ESP, means_SNR] = decode_json(fname);

ESP = ESP(distances==2);
RSSI = RSSI(distances==2);


%% PLOTTING SECTION
figure();
plot(ESP, 'r'); grid on; hold on;
plot(RSSI, 'b');
xlabel('Measure number [-]');
ylabel('Signal [dBm]');
title('Signal received');
legend('ESP', 'RSSI');

figure();
histfit(ESP); grid on;
xlabel('Signal [dBm]');
ylabel('Occurences [-]');
title('Histogram fit for signal');


%% STAT SECTION
[H, p] = jbtest(ESP);       % result: does not follow it...