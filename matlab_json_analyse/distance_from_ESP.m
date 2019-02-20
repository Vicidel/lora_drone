function [distance, confidence_interval_95] = distance_from_ESP(ESP)

% load interpolation data
load('interp_polynom_ESP.mat', 'interpolation_polynom_ESP');
p = interpolation_polynom_ESP;
ci = confint(p);

% get results
distance = ESP^2 * p.p1 + ESP * p.p2 + p.p3;
confidence_interval_95(:) = ESP^2 * ci(:,1) + ESP * ci(:,2) + ci(:,3);

end

