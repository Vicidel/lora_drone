clear all; close all;

% get data
filename_45 = '../json_backup/20190215pm_antenna45.json';
filename_up = '../json_backup/20190215pm_antennaup.json';
filename_hor = '../json_backup/20190215pm_antennahor.json';
[time45, distances45, ~, RSSI45, ESP45, ~, means_RSSI45, means_ESP45, ~] = decode_json(filename_45);
[timeup, distancesup, ~, RSSIup, ESPup, ~, means_RSSIup, means_ESPup, ~] = decode_json(filename_up);
[timehor, distanceshor, ~, RSSIhor, ESPhor, ~, means_RSSIhor, means_ESPhor, ~] = decode_json(filename_hor);

% calibration distances
d_calib = [10 20 50 100 150 200];


% plot mean ESP and RSSI against distances
figure();
plot(d_calib(1:5), means_ESP45(1:5), 'ro-'); hold on; grid on;
plot(d_calib(1:5), means_ESPup(1:5), 'bo-');
plot(d_calib(1:5), means_ESPhor(1:5), 'go-');
legend('45', 'up', 'hor');
xlabel('Distance [m]');
ylabel('ESP [dBm]');
title('Mean ESP');

% % boxplot for 45°
% figure();
% group = [repmat(d_calib(1), length(ESP45(distances45==10)), 1);...
%         repmat(d_calib(2), length(ESP45(distances45==20)), 1);...
%         repmat(d_calib(3), length(ESP45(distances45==50)), 1);...
%         repmat(d_calib(4), length(ESP45(distances45==100)), 1);...
%         repmat(d_calib(5), length(ESP45(distances45==150)), 1)];
% boxplot([ESP45(distances45==10); ESP45(distances45==20); ESP45(distances45==50); ...
%         ESP45(distances45==100); ESP45(distances45==150)], ...
%         group, 'positions', d_calib(1:5), 'labels', d_calib(1:5)); grid on;
% xlabel('Distance [m]');
% ylabel('ESP [dBm]');
% title('Boxplot of ESP for 45° antenna');
% 
% % boxplot for horizontal
% figure();
% group = [repmat(d_calib(1), length(ESPhor(distanceshor==10)), 1);...
%         repmat(d_calib(2), length(ESPhor(distanceshor==20)), 1);...
%         repmat(d_calib(3), length(ESPhor(distanceshor==50)), 1);...
%         repmat(d_calib(4), length(ESPhor(distanceshor==100)), 1);...
%         repmat(d_calib(5), length(ESPhor(distanceshor==150)), 1)];
% boxplot([ESPhor(distanceshor==10); ESPhor(distanceshor==20); ESPhor(distanceshor==50); ...
%         ESPhor(distanceshor==100); ESPhor(distanceshor==150)], ...
%         group, 'positions', d_calib(1:5), 'labels', d_calib(1:5)); grid on; 
% xlabel('Distance [m]');
% ylabel('ESP [dBm]');
% title('Boxplot of ESP for horizontal antenna');
% 
% % boxplot for vertical
% figure();
% group = [repmat(d_calib(1), length(ESPup(distancesup==10)), 1);...
%         repmat(d_calib(2), length(ESPup(distancesup==20)), 1);...
%         repmat(d_calib(3), length(ESPup(distancesup==50)), 1);...
%         repmat(d_calib(4), length(ESPup(distancesup==100)), 1);...
%         repmat(d_calib(5), length(ESPup(distancesup==150)), 1)];
% boxplot([ESPup(distancesup==10); ESPup(distancesup==20); ESPup(distancesup==50); ...
%         ESPup(distancesup==100); ESPup(distancesup==150)], ...
%         group, 'positions', d_calib(1:5), 'labels', d_calib(1:5)); grid on;
% xlabel('Distance [m]');
% ylabel('ESP [dBm]');
% title('Boxplot of ESP for vertical antenna');

% trying aboxplot function: http://alex.bikfalvi.com/research/advanced_matlab_boxplot/
figure();
x1 = ESP45(distances45==10); x1 = x1(1:68);
x2 = ESP45(distances45==20); x2 = x2(1:68);
x3 = ESP45(distances45==50); x3 = x3(1:68);
x4 = ESP45(distances45==100); x4 = x4(1:68);
x5 = ESP45(distances45==150); x5 = x5(1:68);
x = cat(2, x1, x2, x3, x4, x5);
y1 = ESPup(distancesup==10); y1 = y1(1:68);
y2 = ESPup(distancesup==20); y2 = y2(1:68);
y3 = ESPup(distancesup==50); y3 = y3(1:68);
y4 = ESPup(distancesup==100); y4 = y4(1:68);
y5 = ESPup(distancesup==150); y5 = y5(1:68);
y = cat(2, y1, y2, y3, y4, y5);
z1 = ESPup(distancesup==10); z1 = z1(1:68);
z2 = ESPup(distancesup==20); z2 = z2(1:68);
z3 = ESPhor(distanceshor==50); z3 = z3(1:68);
z4 = ESPhor(distanceshor==100); z4 = z4(1:68);
z5 = ESPhor(distanceshor==150); z5 = [z5; z5]; z5 = z5(1:68);
z = cat(2, z1, z2, z3, z4, z5);
h = cat(1, reshape(x,[1 size(x)]), reshape(y,[1 size(y)]), reshape(z,[1 size(z)]));
aboxplot(h,'labels',[10, 20, 50, 100, 150]); grid on;
xlabel('Distance [m]');
ylabel('ESP [dBm]');
title('Boxplot of ESP');
legend('45°', 'upward', 'horizontal');

% plot of stdev
figure();
mean_std_45 = mean([std(x1), std(x2), std(x3), std(x4), std(x5)]);
mean_std_up = mean([std(y1), std(y2), std(y3), std(y4), std(y5)]);
mean_std_hor = mean([std(z1), std(z2), std(z3), std(z4), std(z5)]);
plot([1, 2, 3], [mean_std_45, mean_std_up, mean_std_hor], 'o'); grid on;
xticks([1 2 3]); xticklabels({'45°', 'upward', 'horizontal'});
xlabel('Antenna orientation');
ylabel('STD of ESP [dBm]');
title('Standard deviation of ESP as function of antenna orientation');