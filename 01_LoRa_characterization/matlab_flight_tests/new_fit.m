%%
clear all;

% init total variables
tot_ESP = [];
tot_RSSI = [];
tot_dist = [];
tot_angle_deg = [];

% get data
fnames = ['matlab_flight_tests/20190619-1355-tri-data.json'; 'matlab_flight_tests/20190619-1417-tri-data.json'];

% for each file
for j=1: length(fnames(:,1))
    db = jsondecode(fileread(fnames(j,:)));

    % extract x,y,z,d
    x = zeros(1,length(fieldnames(db)));
    y = zeros(1,length(fieldnames(db)));
    z = zeros(1,length(fieldnames(db)));
    dist = zeros(1,length(fieldnames(db)));
    ESP = zeros(1,length(fieldnames(db)));
    RSSI = zeros(1,length(fieldnames(db)));
    for i=1: length(fieldnames(db))
        datapoint_name = strcat('datapoint_', num2str(i-1));
        datapoint = db.(datapoint_name);
        x(i) = datapoint.x;
        y(i) = datapoint.y;
        z(i) = datapoint.z;
        dist(i) = datapoint.distance;
        ESP(i) = datapoint.esp;
        RSSI(i) = datapoint.rssi;
    end

    % correct with distance between 0 and home
    x = x - x(1);
    y = y - y(1);

    % correct with distance between home and node
    x = x + 11.89;
    y = y - 4.21;

    % get real distance
    real_dist = zeros(1,length(fieldnames(db)));
    for i=1: length(fieldnames(db))
        real_dist(i) = sqrt(x(i)^2+y(i)^2+z(i)^2);
    end

    % get angle
    real_angle_rad = zeros(1,length(fieldnames(db)));
    real_angle_deg = zeros(1,length(fieldnames(db)));
    for i=1: length(fieldnames(db))
        real_angle_rad(i) = atan(z(i)/sqrt(x(i)^2+y(i)^2));
    end
    real_angle_deg = real_angle_rad*180/pi;

    % append data to total
    tot_ESP = [tot_ESP, ESP];
    tot_RSSI = [tot_RSSI, RSSI];
    tot_dist = [tot_dist, real_dist];
    tot_angle_deg = [tot_angle_deg, real_angle_deg];
end

%%
close all;

% plot dist and angle
figure;
plot(tot_angle_deg, 'o'); hold on; grid on;
plot(tot_dist, 'o');
title('Angle and distance recorded');
legend('Angle [deg]', 'Distance [m]');

% plot signal
figure;
plot(tot_ESP, 'o'); hold on; grid on;
plot(tot_RSSI, 'o');
title('Signal recorded');
legend('ESP', 'RSSI');

%% 
% correct signal with angle attenuation
tot_ESP_corr = tot_ESP;
tot_RSSI_corr = tot_RSSI;
for i=1: length(tot_ESP)
    tot_ESP_corr(i) = tot_ESP_corr(i) - func_attenuation_angle(tot_angle_deg(i));
    tot_RSSI_corr(i) = tot_RSSI_corr(i) - func_attenuation_angle(tot_angle_deg(i));
end

% plot signal
figure;
plot(tot_ESP_corr, 'o'); hold on; grid on;
plot(tot_RSSI_corr, 'o');
title('Corrected signal recorded');
legend('ESP', 'RSSI');

%%
% plot
figure;
plot(tot_dist, tot_ESP_corr, 'o'); grid on; hold on;
plot(tot_dist, tot_ESP, 'o');
legend('ESP corr', 'ESP');
title('Signal as function of distance');
