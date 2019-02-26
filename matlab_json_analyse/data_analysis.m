%% DECODING SECTION
clear all; close all;

% opens the JSON file decodes it
fname = '../json_backup/20190219pm_data.json';
[time, distances, SF, RSSI, ESP, SNR, means_RSSI, means_ESP, means_SNR] = decode_json(fname);

% calibration distances
d_calib = [10 20 50 100 150 200];

% number of samples
nb_10 = sum(distances==10);
nb_20 = sum(distances==20);
nb_50 = sum(distances==50);
nb_100 = sum(distances==100);
nb_150 = sum(distances==150);
nb_200 = sum(distances==200);

% std = noise level
noises_ESP = zeros(size(d_calib));
noises_RSSI = zeros(size(d_calib));
for i=1: length(d_calib)
    noises_ESP(i) = std(ESP(distances==d_calib(i)));
    noises_RSSI(i) = std(RSSI(distances==d_calib(i)));
end

%% PLOTTING SECTION
% plotting ESP against distance
figure();
plot(distances, ESP, 'x'); grid on;
xlabel('Real distance [m]');
ylabel('ESP [dBm]');
title('ESP function of distance');

% plotting SNR against distance
figure();
plot(distances, SNR, 'x'); grid on;
xlabel('Real distance [m]');
ylabel('SNR [-]');
title('SNR function of distance');

% plotting ESP against time
figure();
plot(time(distances==10), ESP(distances==10), 'rx'); grid on; hold on;
plot(time(distances==20), ESP(distances==20), 'bx');
plot(time(distances==50), ESP(distances==50), 'gx');
plot(time(distances==100), ESP(distances==100), 'yx');
plot(time(distances==150), ESP(distances==150), 'cx');
plot(time(distances==200), ESP(distances==200), 'mx');
xlabel('Time [s]');
ylabel('ESP [dBm]');
legend('10m', '20m', '50m', '100m', '150m', '200m');
title('ESP function of time');

% plot mean ESP and RSSI against distances
figure();
plot(d_calib, means_ESP, 'ro-'); hold on; grid on;
plot(d_calib, means_RSSI, 'bo-');
legend('ESP', 'RSSI');
xlabel('Distance [m]');
ylabel('ESP/RSSI [dBm]');
title('Mean ESP and RSSI');

% plot mean SNR against distances
figure();
plot(d_calib, means_SNR, 'o-'); grid on
xlabel('Distance [m]');
ylabel('SNR [-]');
title('Mean SNR');

% boxplot of ESP
figure();
boxplot([ESP(distances==10); ESP(distances==20); ESP(distances==50); ...
        ESP(distances==100); ESP(distances==150); ESP(distances==200)], ...
        distances, 'positions', d_calib, 'labels', d_calib, 'Width', 10); grid on; 
xlabel('Distance [m]');
ylabel('ESP [dBm]');
title('Boxplot of ESP');

% boxplot of SNR
figure();
boxplot([SNR(distances==10); SNR(distances==20); SNR(distances==50); ...
        SNR(distances==100); SNR(distances==150); SNR(distances==200)], ...
        distances, 'positions', d_calib, 'labels', d_calib, 'Width', 10); grid on; 
xlabel('Distance [m]');
ylabel('SNR [-]');
title('Boxplot of SNR');

% boxplot of RSSI
figure();
boxplot([RSSI(distances==10); RSSI(distances==20); RSSI(distances==50); ...
        RSSI(distances==100); RSSI(distances==150); RSSI(distances==200)], ...
        distances, 'positions', d_calib, 'labels', d_calib, 'Width', 10); grid on; 
xlabel('Distance [m]');
ylabel('RSSI [dBm]');
title('Boxplot of RSSI');

% plot ESP with SF
figure();
plot(distances(SF==7), ESP(SF==7), 'ro'); grid on; hold on;
% plot(distances(SF==8), ESP(SF==8), 'go');
plot(distances(SF==9), ESP(SF==9), 'bo');
plot(distances(SF==10), ESP(SF==10), 'go');
% plot(distances(SF==11), ESP(SF==11), 'co');
plot(distances(SF==12), ESP(SF==12), 'ko');
xlabel('Real distance [m]');
ylabel('ESP [dBm]');
legend('SF7', 'SF9', 'SF10', 'SF12');
title('ESP function of distance and SF');

% plot noise level against distance
figure();
plot(d_calib, noises_ESP, 'ro-'); hold on; grid on;
plot(d_calib, noises_RSSI, 'bo-');
legend('ESP', 'RSSI');
xlabel('Distance [m]');
ylabel('ESP/RSSI [dBm]');
title('Noise of ESP and RSSI');

%% ANALYSIS SECTION

% remove all the 10
fit_dist = distances(nb_10+1:end);     
fit_ESP = ESP(nb_10+1:end);
fit_RSSI = RSSI(nb_10+1:end);

% fit ESP as function of distance
x = fit_dist; y = fit_ESP;
[fitresult_dESP, gof_dESP] = fit(x, y, fittype('poly2'));
figure();
plot(fitresult_dESP, x, y, 'x'); grid on;
legend('ESP signal', 'Polynomial fit');
xlabel('Distance [m]');
ylabel('ESP [dBm]');
title('Fit of ESP as function of distance');
save('polynom_dist_to_ESP.mat', 'fitresult_dESP');

% fit RSSI as function of distance
x = fit_dist; y = fit_RSSI;
[fitresult_dRSSI, gof_dRSSI] = fit(x, y, fittype('poly2'));
figure();
plot(fitresult_dRSSI, x, y, 'x'); grid on;
legend('RSSI signal', 'Polynomial fit');
xlabel('Distance [m]');
ylabel('RSSI [dBm]');
title('Fit of RSSI as function of distance');
save('polynom_dist_to_RSSI.mat', 'fitresult_dRSSI');

% fit distance as function of ESP 
x = fit_ESP; y = fit_dist;
[fitresult_ESPd, gof_ESPd] = fit(x, y, fittype('poly2'));
figure();
plot(fitresult_ESPd, x, y, 'x'); grid on;
legend('ESP signal', 'Polynomial fit 2');
ylabel('Distance [m]');
xlabel('ESP [dBm]');
title('Fit of distance as function of ESP');
save('polynom_ESP_to_dist.mat', 'fitresult_ESPd');

% fit distance as function of RSSI 
x = fit_RSSI; y = fit_dist;
[fitresult_RSSId, gof_RSSId] = fit(x, y, fittype('poly2'));
figure();
plot(fitresult_RSSId, x, y, 'x'); grid on;
legend('RSSI signal', 'Polynomial fit 2');
ylabel('Distance [m]');
xlabel('RSSI [dBm]');
title('Fit of distance as function of RSSI');
save('polynom_RSSI_to_dist.mat', 'fitresult_RSSId');
