function [distance, confidence_interval_95] = distance_from_RSSI(RSSI)

% load interpolation data
load('interp_polynom_RSSI.mat', 'interpolation_polynom_RSSI');
p = interpolation_polynom_RSSI;
ci = confint(p);

% get results
distance = RSSI^2 * p.p1 + RSSI * p.p2 + p.p3;
confidence_interval_95(:) = RSSI^2 * ci(:,1) + RSSI * ci(:,2) + ci(:,3);

end

