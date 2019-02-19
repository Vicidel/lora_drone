clear all; close all;

% opens the JSON file decodes it
fname = '../json_backup/20190219pm_data.json';
[time, distances, RSSI, ESP, SNR, means_RSSI, means_ESP, means_SNR] = decode_json(fname);

% calibration distances
d_calib = [10 20 50 100 150 200];

% number of samples
nb_10 = sum(distances==10);
nb_20 = sum(distances==20);
nb_50 = sum(distances==50);
nb_100 = sum(distances==100);
nb_150 = sum(distances==150);
nb_200 = sum(distances==200);

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

% plot mean ESP and RSSI against distances
figure();
plot(d_calib, means_ESP, 'ro-'); hold on; grid on;
plot(d_calib, means_RSSI, 'bo-');
legend('ESP', 'RSSI');
xlabel('Distance [m]');
ylabel('ESP/RSSI [dBm]');
title('Mean ESP and RSSI');

% % plot mean SNR against distances
% figure();
% plot(d_calib, means_SNR, 'o-'); grid on
% xlabel('Distance [m]');
% ylabel('SNR [-]');
% title('Mean SNR');

% boxplot of ESP
figure();
group = [repmat(d_calib(1), length(ESP(distances==10)), 1);...
        repmat(d_calib(2), length(ESP(distances==20)), 1);...
        repmat(d_calib(3), length(ESP(distances==50)), 1);...
        repmat(d_calib(4), length(ESP(distances==100)), 1);...
        repmat(d_calib(5), length(ESP(distances==150)), 1);...
        repmat(d_calib(6), length(ESP(distances==200)), 1)];
boxplot([ESP(distances==10); ESP(distances==20); ESP(distances==50); ...
        ESP(distances==100); ESP(distances==150); ESP(distances==200)], ...
        group, 'positions', d_calib, 'labels', d_calib); grid on; 
xlabel('Distance [m]');
ylabel('ESP [dBm]');
title('Boxplot of ESP');

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
% ylabel('SNR [dBm]');
% title('Boxplot of SNR');