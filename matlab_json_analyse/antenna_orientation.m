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

% boxplot for 45°
figure();
group = [repmat(d_calib(1), length(ESP45(distances45==10)), 1);...
        repmat(d_calib(2), length(ESP45(distances45==20)), 1);...
        repmat(d_calib(3), length(ESP45(distances45==50)), 1);...
        repmat(d_calib(4), length(ESP45(distances45==100)), 1);...
        repmat(d_calib(5), length(ESP45(distances45==150)), 1)];
boxplot([ESP45(distances45==10); ESP45(distances45==20); ESP45(distances45==50); ...
        ESP45(distances45==100); ESP45(distances45==150)], ...
        group, 'positions', d_calib(1:5), 'labels', d_calib(1:5)); grid on;
xlabel('Distance [m]');
ylabel('ESP [dBm]');
title('Boxplot of ESP for 45° antenna');

% boxplot for horizontal
figure();
group = [repmat(d_calib(1), length(ESPhor(distanceshor==10)), 1);...
        repmat(d_calib(2), length(ESPhor(distanceshor==20)), 1);...
        repmat(d_calib(3), length(ESPhor(distanceshor==50)), 1);...
        repmat(d_calib(4), length(ESPhor(distanceshor==100)), 1);...
        repmat(d_calib(5), length(ESPhor(distanceshor==150)), 1)];
boxplot([ESPhor(distanceshor==10); ESPhor(distanceshor==20); ESPhor(distanceshor==50); ...
        ESPhor(distanceshor==100); ESPhor(distanceshor==150)], ...
        group, 'positions', d_calib(1:5), 'labels', d_calib(1:5)); grid on; 
xlabel('Distance [m]');
ylabel('ESP [dBm]');
title('Boxplot of ESP for horizontal antenna');

% boxplot for vertical
figure();
group = [repmat(d_calib(1), length(ESPup(distancesup==10)), 1);...
        repmat(d_calib(2), length(ESPup(distancesup==20)), 1);...
        repmat(d_calib(3), length(ESPup(distancesup==50)), 1);...
        repmat(d_calib(4), length(ESPup(distancesup==100)), 1);...
        repmat(d_calib(5), length(ESPup(distancesup==150)), 1)];
boxplot([ESPup(distancesup==10); ESPup(distancesup==20); ESPup(distancesup==50); ...
        ESPup(distancesup==100); ESPup(distancesup==150)], ...
        group, 'positions', d_calib(1:5), 'labels', d_calib(1:5)); grid on;
xlabel('Distance [m]');
ylabel('ESP [dBm]');
title('Boxplot of ESP for vertical antenna');