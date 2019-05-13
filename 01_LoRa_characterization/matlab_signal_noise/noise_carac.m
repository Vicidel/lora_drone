% script to get the noise levels at each distances

%% DECODING SECTION
clear all; close all;

% opens the JSON file decodes it
fname = 'json_backup/20190219pm_data.json';
[time, distances, SF, RSSI, ESP, SNR, means_RSSI, means_ESP, means_SNR] = decode_json(fname);

% calibration distances
d_calib = [10 20 50 100 150 200];

% number of samples
nb_10  = sum(distances==10);
nb_20  = sum(distances==20);
nb_50  = sum(distances==50);
nb_100 = sum(distances==100);
nb_150 = sum(distances==150);
nb_200 = sum(distances==200);

% get noise
signal_d10  = RSSI(distances==10);
noise_d10   = signal_d10 - mean(signal_d10);
signal_d20  = RSSI(distances==20);
noise_d20   = signal_d20 - mean(signal_d20);
signal_d50  = RSSI(distances==50);
noise_d50   = signal_d50 - mean(signal_d50);
signal_d100 = RSSI(distances==100);
noise_d100  = signal_d100 - mean(signal_d100);
signal_d150 = RSSI(distances==150);
noise_d150  = signal_d150 - mean(signal_d150);
signal_d200 = RSSI(distances==200); 
noise_d200  = signal_d200 - mean(signal_d200);

% get normal fir
norm_fit_10  = fitdist(noise_d10, 'Normal');
norm_fit_20  = fitdist(noise_d20, 'Normal');
norm_fit_50  = fitdist(noise_d50, 'Normal');
norm_fit_100 = fitdist(noise_d100, 'Normal');
norm_fit_150 = fitdist(noise_d150, 'Normal');
norm_fit_200 = fitdist(noise_d200, 'Normal');

% store normal coefficients
noise_normal_fit = [norm_fit_10.sigma, norm_fit_20.sigma, norm_fit_50.sigma, ...
                    norm_fit_100.sigma, norm_fit_150.sigma, norm_fit_200.sigma];
save('matlab_signal_noise/noise_normal_sigma.mat', 'noise_normal_fit');

% fit straight line
d_calib_v2 = d_calib(2:end);
noise_normal_fit_v2 = noise_normal_fit(2:end);



%% PLOT SECTION

% figure();
% histfit(noise_d10); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Noise level for d=10m');
% figure();
% histfit(noise_d20); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Noise level for d=20m');
% figure();
% histfit(noise_d50); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Noise level for d=50m');
% figure();
% histfit(noise_d100); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Noise level for d=100m');
% figure();
% histfit(noise_d150); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Noise level for d=150m');
% figure();
% histfit(noise_d200); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Noise level for d=200m');


%% MULTIPLOT SECTION

% figure();
% subplot(2,3,1);
% histfit(noise_d10); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Distance of 10m');
% subplot(2,3,2);
% histfit(noise_d20); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Distance of 20m');
% subplot(2,3,3);
% histfit(noise_d50); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Distance of 50m');
% subplot(2,3,4);
% histfit(noise_d100); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Distance of 100m');
% subplot(2,3,5);
% histfit(noise_d150); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Distance of 150m');
% subplot(2,3,6);
% histfit(noise_d200); grid on;
% xlabel('Deviation from mean [dBm]');
% ylabel('Occurences');
% title('Distance of 200m');

figure();
h(1) = subplot(2,3,1);
histfit(noise_d20); grid on;
xlabel('Deviation from mean [dBm]');
ylabel('Occurences');
title('Distance of 20m');
h(2) = subplot(2,3,2);
histfit(noise_d50); grid on;
xlabel('Deviation from mean [dBm]');
ylabel('Occurences');
title('Distance of 50m');
h(3) = subplot(2,3,3);
histfit(noise_d100); grid on;
xlabel('Deviation from mean [dBm]');
ylabel('Occurences');
title('Distance of 100m');
h(4) = subplot(2,3,4);
histfit(noise_d150); grid on;
xlabel('Deviation from mean [dBm]');
ylabel('Occurences');
title('Distance of 150m');
h(5) = subplot(2,3,5);
histfit(noise_d200); grid on;
xlabel('Deviation from mean [dBm]');
ylabel('Occurences');
title('Distance of 200m');
pos = get(h, 'Position');
new = mean(cellfun(@(v)v(1),pos(1:2)));
set(h(4),'Position',[new,pos{end}(2:end)])
new = mean(cellfun(@(v)v(1),pos(2:3)));
set(h(5),'Position',[new,pos{end}(2:end)])