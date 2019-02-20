function [ESP_new, RSSI_new] = SF_correction(SF, ESP, RSSI)

switch SF      % correction based on SF
    case 7
        ESP_new = ESP;
        RSSI_new = RSSI;
    case 8
        ESP_new = ESP - 2.4;
        RSSI_new = RSSI - 2.4;
    case 9
        ESP_new = ESP - 4.9;
        RSSI_new = RSSI - 4.9;
    case 10
        ESP_new = ESP - 7.5;
        RSSI_new = RSSI - 7.5;
    case 11
        ESP_new = ESP - 10;
        RSSI_new = RSSI - 10;
    case 12
        ESP_new = ESP - 12.7;
        RSSI_new = RSSI - 12.7;
end

end

