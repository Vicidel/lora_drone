clear all; close all;

% opens the JSON file decodes it
fname = '20190215am_SF7.json';
[time, distances, RSSI, ESP, means_RSSI, means_ESP] = decode_json(fname);

% calibration distances
d_calib = [10 20 50 100 150 200];


% plotting RSSI against distance
figure();
plot(distances, RSSI, 'x');
xlabel('Real distance [m]');
ylabel('RSSI [dBm]');
title('RSSI function of distance');

% plotting ESP against distance
figure();
plot(distances, ESP, 'x');
xlabel('Real distance [m]');
ylabel('ESP [dBm]');
title('ESP function of distance');

% plotting RSSI against time
figure();
plot(time, RSSI, 'x');
xlabel('Time [s]');
ylabel('RSSI [dBm]');
title('RSSI function of time');

% plotting ESP against time
figure();
plot(time, ESP, 'x');
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

% plot average time between two measures
figure();
plot(time(2:end)-time(1:end-1));
xlabel('Message number [-]');
ylabel('Time [s]');
title('Time between two messages');
