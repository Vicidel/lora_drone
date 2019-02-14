% opens the JSON file and stores it in variable
fname = 'database2.json';
db = jsondecode(fileread(fname));

% to take only values from on gateway
gateway_we_want = "004A1092";



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%  DECODING SECTION  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% get values
number_message = length(db);
distances = zeros(number_message, 1);
RSSI = zeros(number_message, 1);
ESP  = zeros(number_message, 1);
SNR  = zeros(number_message, 1);
time = zeros(number_message, 1);
for i=1: number_message
    gateway_index = 0;
    distances(i) = db(i).real_dist;
    for j=1: length(db(i).gateway_id(:))
        if db(i).gateway_id(j) == gateway_we_want
            gateway_index = j;
        end
    end
    if gateway_index ~= 0
        RSSI(i) = db(i).gateway_rssi(gateway_index);
        SNR(i)  = db(i).gateway_snr(gateway_index);
        ESP(i)  = db(i).gateway_esp(gateway_index);  
        time(i) = db(i).timestamp.x_date;
    end
end

% normalize time to be in seconds, staring from first collection
time = (time - time(1))/1000;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%  PLOTTING SECTION  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % plotting RSSI against distance
% figure();
% plot(distances, RSSI, 'x');
% xlabel('Real distance [m]');
% ylabel('RSSI [dBm]');
% title('RSSI function of distance');

% plotting ESP against distance
figure();
plot(distances, ESP, 'x');
xlabel('Real distance [m]');
ylabel('ESP [dBm]');
title('ESP function of distance');

% % plotting RSSI against time
% figure();
% plot(time, RSSI, 'x');
% xlabel('Time [s]');
% ylabel('RSSI [dBm]');
% title('RSSI function of time');
% 
% % plotting ESP against time
% figure();
% plot(time, ESP, 'x');
% xlabel('Time [s]');
% ylabel('ESP [dBm]');
% title('ESP function of time');
