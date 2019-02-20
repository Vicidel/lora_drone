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
% % plotting ESP against distance
% figure();
% plot(distances, ESP, 'x'); grid on;
% xlabel('Real distance [m]');
% ylabel('ESP [dBm]');
% title('ESP function of distance');

% % plotting SNR against distance
% figure();
% plot(distances, SNR, 'x'); grid on;
% xlabel('Real distance [m]');
% ylabel('SNR [-]');
% title('SNR function of distance');

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
title('ESP function of time');

% % plot mean ESP and RSSI against distances
% figure();
% plot(d_calib, means_ESP, 'ro-'); hold on; grid on;
% plot(d_calib, means_RSSI, 'bo-');
% legend('ESP', 'RSSI');
% xlabel('Distance [m]');
% ylabel('ESP/RSSI [dBm]');
% title('Mean ESP and RSSI');

% % plot mean SNR against distances
% figure();
% plot(d_calib, means_SNR, 'o-'); grid on
% xlabel('Distance [m]');
% ylabel('SNR [-]');
% title('Mean SNR');

% % boxplot of ESP
% figure();
% group = [repmat(d_calib(1), length(ESP(distances==10)), 1);...
%         repmat(d_calib(2), length(ESP(distances==20)), 1);...
%         repmat(d_calib(3), length(ESP(distances==50)), 1);...
%         repmat(d_calib(4), length(ESP(distances==100)), 1);...
%         repmat(d_calib(5), length(ESP(distances==150)), 1);...
%         repmat(d_calib(6), length(ESP(distances==200)), 1)];
% boxplot([ESP(distances==10); ESP(distances==20); ESP(distances==50); ...
%         ESP(distances==100); ESP(distances==150); ESP(distances==200)], ...
%         group, 'positions', d_calib, 'labels', d_calib); grid on; 
% xlabel('Distance [m]');
% ylabel('ESP [dBm]');
% title('Boxplot of ESP');

% % boxplot of SNR
% figure();
% group = [repmat(d_calib(1), length(SNR(distances==10)), 1);...
%         repmat(d_calib(2), length(SNR(distances==20)), 1);...
%         repmat(d_calib(3), length(SNR(distances==50)), 1);...
%         repmat(d_calib(4), length(SNR(distances==100)), 1);...
%         repmat(d_calib(5), length(SNR(distances==150)), 1);...
%         repmat(d_calib(6), length(SNR(distances==200)), 1)];
% boxplot([SNR(distances==10); SNR(distances==20); SNR(distances==50); ...
%         SNR(distances==100); SNR(distances==150); SNR(distances==200)], ...
%         group, 'positions', d_calib, 'labels', d_calib); grid on; 
% xlabel('Distance [m]');
% ylabel('SNR [-]');
% title('Boxplot of SNR');

% % plot ESP with SF
% figure();
% plot(distances(SF==7), ESP(SF==7), 'ro'); grid on; hold on;
% plot(distances(SF==9), ESP(SF==9), 'go');
% plot(distances(SF==10), ESP(SF==10), 'yo');
% plot(distances(SF==12), ESP(SF==12), 'co');
% xlabel('Real distance [m]');
% ylabel('ESP [dBm]');
% title('ESP function of distance and SF (red=7, green=9...)');

% plot noise level against distance
figure();
plot(d_calib, noises_ESP, 'ro-'); hold on; grid on;
plot(d_calib, noises_RSSI, 'bo-');
legend('ESP', 'RSSI');
xlabel('Distance [m]');
ylabel('ESP/RSSI [dBm]');
title('Noise of ESP and RSSI');

%% ANALYSIS SECTION
% % fit ESP as function of distance
% x = distances(nb_10+1:end);     % remove all the 10
% y = ESP(nb_10+1:end);
% [fitresult, gof] = fit(x, y, fittype('poly2'));
% figure();
% plot(fitresult, x, y, 'x'); grid on;
% legend('ESP signal', 'Polynomial fit');
% xlabel('Distance [m]');
% ylabel('ESP [dBm]');
% title('Polynomial interpolation of ESP');
% interpolation_polynom = fitresult;
% save('interp_polynom.mat', 'interpolation_polynom');

% fit distance as function of ESP 
y_dist = distances(nb_10+1:end);     % remove all the 10
x_ESP = ESP(nb_10+1:end);
[fitresult_ESP, gof_ESP] = fit(x_ESP, y_dist, fittype('poly2'));
figure();
plot(fitresult_ESP, x_ESP, y_dist, 'x'); grid on;
legend('ESP signal', 'Polynomial fit 2');
ylabel('Distance [m]');
xlabel('ESP [dBm]');
title('Polynomial interpolation of ESP');
interpolation_polynom_ESP = fitresult_ESP;
save('interp_polynom_ESP.mat', 'interpolation_polynom_ESP');

% get distance from ESP
[distance_ESP, confidence_interval_ESP] = distance_from_ESP(-102.2);

% fit distance as function of RSSI 
y_dist = distances(nb_10+1:end);     % remove all the 10
x_RSSI = RSSI(nb_10+1:end);
[fitresult_RSSI, gof_RSSI] = fit(x_RSSI, y_dist, fittype('poly2'));
figure();
plot(fitresult_RSSI, x_RSSI, y_dist, 'x'); grid on;
legend('RSSI signal', 'Polynomial fit 2');
ylabel('Distance [m]');
xlabel('RSSI [dBm]');
title('Polynomial interpolation of RSSI');
interpolation_polynom_RSSI = fitresult_RSSI;
save('interp_polynom_RSSI.mat', 'interpolation_polynom_RSSI');

% get distance from RSSI
[distance_RSSI, confidence_interval_RSSI] = distance_from_RSSI(-101);