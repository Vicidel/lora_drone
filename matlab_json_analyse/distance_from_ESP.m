function [distance, confidence_interval_95] = distance_from_ESP(ESP)

% load interpolation data
load('polynom_ESP_to_dist.mat', 'fitresult_ESPd');
p = fitresult_ESPd;
ci = confint(p);

% get results
distance = ESP^2 * p.p1 + ESP * p.p2 + p.p3;
confidence_interval_95(:) = ESP^2 * ci(:,1) + ESP * ci(:,2) + ci(:,3);

end

