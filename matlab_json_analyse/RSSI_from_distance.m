function [RSSI, confidence_interval_95] = RSSI_from_distance(distance)

% load interpolation data
load('polynom_dist_to_RSSI.mat', 'fitresult_dRSSI');
p = fitresult_dRSSI;
ci = confint(p);

% get results
RSSI = distance^2 * p.p1 + distance * p.p2 + p.p3;
confidence_interval_95(:) = distance^2 * ci(:,1) + distance * ci(:,2) + ci(:,3);

end

