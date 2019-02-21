function [RSSI, confidence_interval_95] = RSSI_from_distance(distance, p)

ci = confint(p);

% get results
RSSI = distance^2 * p.p1 + distance * p.p2 + p.p3;
confidence_interval_95(:) = distance^2 * ci(:,1) + distance * ci(:,2) + ci(:,3);

end

