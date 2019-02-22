function [ESP, confidence_interval_95] = ESP_from_distance(distance, p)

ci = confint(p);

if distance > 190       % to remove re-growth of parabola after certain distance
    distance = 190;
end

% get results
ESP = distance^2 * p.p1 + distance * p.p2 + p.p3;
confidence_interval_95(:) = distance^2 * ci(:,1) + distance * ci(:,2) + ci(:,3);

end

