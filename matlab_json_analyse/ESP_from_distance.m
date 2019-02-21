function [ESP, confidence_interval_95] = ESP_from_distance(distance)

% load interpolation data
load('polynom_dist_to_ESP.mat', 'fitresult_dESP');
p = fitresult_dESP;
ci = confint(p);

% get results
ESP = distance^2 * p.p1 + distance * p.p2 + p.p3;
confidence_interval_95(:) = distance^2 * ci(:,1) + distance * ci(:,2) + ci(:,3);

end

