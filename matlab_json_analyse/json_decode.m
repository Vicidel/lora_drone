% opens the JSON file and stores it in variable
fname = 'database2.json';
db = jsondecode(fileread(fname));

% to take only values from on gateway
gateway_we_want = "004A1092";

% calibration distances
d_calib = [10 20 50 100 150 200];



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

% get mean ESP and RSSI
means_ESP = [mean(ESP(distances==10)) mean(ESP(distances==20)) mean(ESP(distances==50))...
             mean(ESP(distances==100)) mean(ESP(distances==150)) mean(ESP(distances==200))]';
means_RSSI = [mean(RSSI(distances==10)) mean(RSSI(distances==20)) mean(RSSI(distances==50))...
             mean(RSSI(distances==100)) mean(RSSI(distances==150)) mean(RSSI(distances==200))]';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%  PLOTTING SECTION  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % plotting RSSI against distance
% figure();
% plot(distances, RSSI, 'x');
% xlabel('Real distance [m]');
% ylabel('RSSI [dBm]');
% title('RSSI function of distance');

% % plotting ESP against distance
% figure();
% plot(distances, ESP, 'x');
% xlabel('Real distance [m]');
% ylabel('ESP [dBm]');
% title('ESP function of distance');

% % plotting RSSI against time
% figure();
% plot(time, RSSI, 'x');
% xlabel('Time [s]');
% ylabel('RSSI [dBm]');
% title('RSSI function of time');

% % plotting ESP against time
% figure();
% plot(time, ESP, 'x');
% xlabel('Time [s]');
% ylabel('ESP [dBm]');
% title('ESP function of time');

% % plot mean ESP and RSSI against distances
% figure();
% plot(d_calib, means_ESP, 'ro-'); hold on; grid on;
% plot(d_calib, means_RSSI, 'bo-');
% legend('ESP', 'RSSI');
% xlabel('Distance [m]');
% ylabel('ESP/RSSI [dBm]');
% title('Mean ESP and RSSI');
