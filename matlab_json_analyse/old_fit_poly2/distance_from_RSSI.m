function [distance, confidence_interval_95] = distance_from_RSSI(RSSI, p)

ci = confint(p);

% get results
distance = RSSI^2 * p.p1 + RSSI * p.p2 + p.p3;
confidence_interval_95(:) = RSSI^2 * ci(:,1) + RSSI * ci(:,2) + ci(:,3);

end

