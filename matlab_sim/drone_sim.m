clear all; close all;

% define node coordinates xyz (z altitude)
node_position = [rand*200-100, rand*200-100, 0];
drone_position = [0, 0, 10];
distance = 9999;

% define increment in position
dist_increment = 10;
increment_0p0 = [0, dist_increment, 0];
increment_0m0 = [0, -dist_increment, 0];
increment_p00 = [dist_increment, 0, 0];
increment_m00 = [-dist_increment, 0, 0];

% define polynom of ESP as function of distance
p = [0.000916, -0.3961, -84.94];    % note that min at distance of 216: https://www.wolframalpha.com/input/?i=0.000916x%5E2-0.3961x-84.94

% init
time = 0; step = 1;
state = 0;
ESP = []; distance = [];

% localization
while time < 400
    
    % plot positions
    plot3(node_position(1), node_position(2), node_position(3), 'ro', 'MarkerSize', 10); grid on; hold on;
    plot3(drone_position(1), drone_position(2), drone_position(3), 'bo', 'MarkerSize', 10);
    view(0, 90);
    
    % recompute distance and get ESP (only temporary, will be from lora message afterwards
    distance = [distance, norm(drone_position - node_position)];
    ESP = [ESP, p(1)*distance(step)^2 + p(2)*distance(step) + p(3)];
    
    switch state
        case 0 % starting point
            state = 1;
        case 1 % try p00 direction
            drone_position = drone_position + increment_p00;
            state = 2;
        case 2 % check progress
            if ESP(step) > ESP(step-1) % better signal
                state = 1;
            else
                state = 3;
            end
        case 3 % try m00 direction
            drone_position = drone_position + increment_m00;
            state = 4;
        case 4 %check progress
            if ESP(step) > ESP(step-1) % better signal
                state = 3;
            else
                state = 5;
            end
        case 5 % try 0p0 direction
            drone_position = drone_position + increment_0p0;
            state = 6;
        case 6 % check progress
            if ESP(step) > ESP(step-1) % better signal
                state = 5;
            else
                state = 7;
            end
        case 7 % try m00 direction
            drone_position = drone_position + increment_0m0;
            state = 8;
        case 8 %check progress
            if ESP(step) > ESP(step-1) % better signal
                state = 7;
            else
                state = 9;
            end
        case 9
            dist_increment = dist_increment/2;
            increment_0p0 = [0, dist_increment, 0];
            increment_0m0 = [0, -dist_increment, 0];
            increment_p00 = [dist_increment, 0, 0];
            increment_m00 = [-dist_increment, 0, 0];
            state = 1;
            if dist_increment < 2
                break;
            end
    end
    
    % time update
    time = time + 1;
    step = step + 1;
    
    % slows down simulation
    pause(0.25)
end

% estimated final position
estimated_position = drone_position + increment_p00 + increment_0p0;

% plot positions
plot3(node_position(1), node_position(2), node_position(3), 'ro', 'MarkerSize', 10); grid on; hold on;
plot3(drone_position(1), drone_position(2), drone_position(3), 'bo', 'MarkerSize', 10);
plot3(estimated_position(1), estimated_position(2), estimated_position(3), 'gx', 'MarkerSize', 10);
view(0, 90);

% print
fprintf('Estimated position of node: x=%.2f, y=%.2f\n', estimated_position(1), estimated_position(2));
fprintf('Real position of node: x=%.2f, y=%.2f\n', node_position(1), node_position(2));
fprintf('Error: dx=%.2f, dy=%.2f, norm=%.2f\n', abs(estimated_position(1) - node_position(1)), abs(estimated_position(2) - node_position(2)), norm([abs(drone_position(1) - node_position(1)), abs(drone_position(2) - node_position(2))])); 