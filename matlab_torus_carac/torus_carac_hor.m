%% DECODING SECTION
clear all; close all;

% opens the JSON file decodes it
fname = 'json_backup/20190308pm_torus_hor_indoor_v7.json';
[time, angles, SF, RSSI, ESP, SNR, ~, ~, ~, tx_pow] = decode_json(fname);

% angles
angles_list = [0, 45, 90, 135, 180, 225, 270, 315];

% % number of messages received
% nb_0 = sum(angles==1);
% nb_45 = sum(angles==2);
% nb_90 = sum(angles==3);
% nb_135 = sum(angles==4);
% nb_180 = sum(angles==5);
% nb_225 = sum(angles==6);
% nb_270 = sum(angles==7);
% nb_315 = sum(angles==8);


%% PLOTTING SECTION

% plot ESP against angles
figure();
plot(time, ESP, 'r'); grid on; hold on;
plot(time, RSSI, 'b');
title(fname);
