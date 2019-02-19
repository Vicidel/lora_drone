function [distance, confidence_interval] = distance_from_ESP(ESP)

% load interpolation data
load('interp_polynom.mat');
p = interpolation_polynom;
ci = confint(p);

% get results
distance = ESP^2 * p(1) + ESP * p(2) + p(3);
confidence_interval(:) = ESP^2 * p(:,1) + ESP * p(:,2) + p(:,3);

end

