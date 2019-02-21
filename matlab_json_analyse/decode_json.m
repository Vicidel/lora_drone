function [time, distances, SF, RSSI, ESP, SNR, means_RSSI, means_ESP, means_SNR] = decode_json(filename)
% this function takes as input a filename and returns

% opens the JSON file and stores it in variable
db = jsondecode(fileread(filename));

% to take only values from on gateway
gateway_we_want = "004A1092";



%%%%%%%%%%%%%%%%%%  DECODING SECTION  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% get values
number_message = length(db);
distances = zeros(number_message, 1);
RSSI = zeros(number_message, 1);
ESP  = zeros(number_message, 1);
SNR  = zeros(number_message, 1);
SF   = zeros(number_message, 1);
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
        SF(i)   = db(i).sp_fact;
    end
end

% normalize time to be in seconds, staring from first collection
time = (time - time(1))/1000;

% get mean ESP and RSSI
means_ESP = [mean(ESP(distances==10)) mean(ESP(distances==20)) mean(ESP(distances==50))...
             mean(ESP(distances==100)) mean(ESP(distances==150)) mean(ESP(distances==200))]';
means_RSSI = [mean(RSSI(distances==10)) mean(RSSI(distances==20)) mean(RSSI(distances==50))...
             mean(RSSI(distances==100)) mean(RSSI(distances==150)) mean(RSSI(distances==200))]';
means_SNR = [mean(SNR(distances==10)) mean(SNR(distances==20)) mean(SNR(distances==50))...
             mean(SNR(distances==100)) mean(SNR(distances==150)) mean(SNR(distances==200))]';

end

