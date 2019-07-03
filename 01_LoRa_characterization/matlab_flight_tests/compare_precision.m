% init 
close all
clear all

% get data
% fnames = ['matlab_flight_tests/20190619-1417-tri-data.json';       % continuous
%           'matlab_flight_tests/20190627-0930-tri-data.json'];      % classic
% delta = [-4.7, -20.3;      % continuous
%          -8.9, -12.3];     % classic
fnames = ['matlab_flight_tests/20190627-0930-tri-data.json']; delta = [-8.9, -12.3];      % classic

     
% for each file
for j=1: length(fnames(:,1))
    db   = jsondecode(fileread(fnames(j,:)));
    x    = zeros(1, length(fieldnames(db)));
    y    = zeros(1, length(fieldnames(db)));
    z    = zeros(1, length(fieldnames(db)));
    dist = zeros(1, length(fieldnames(db)));
    ESP  = zeros(1, length(fieldnames(db)));
    RSSI = zeros(1, length(fieldnames(db)));
    
    % extract x,y,z,d
    for i=1: length(fieldnames(db))
        datapoint_name = strcat('datapoint_', num2str(i-1));
        datapoint = db.(datapoint_name);
        x(i) = datapoint.x;
        y(i) = datapoint.y;
        z(i) = datapoint.z;
        dist(i) = datapoint.distance;
        ESP(i) = datapoint.esp;
        RSSI(i) = datapoint.rssi;
    end

    % correct with distance between home and node
    x = x - mean(x);
    y = y - mean(y);

    % correct to have 0 at center
    x = x - delta(j,1);
    y = y - delta(j,2);

    % get real distance
    real_dist = zeros(1,length(fieldnames(db)));
    for i=1: length(fieldnames(db))
        real_dist(i) = sqrt(x(i)^2+y(i)^2+z(i)^2);
    end

    % get angle
    real_angle_rad = zeros(1,length(fieldnames(db)));
    for i=1: length(fieldnames(db))
        real_angle_rad(i) = atan(z(i)/sqrt(x(i)^2+y(i)^2));
    end
    real_angle_deg = real_angle_rad*180/pi;

%     % plot distance
%     figure(1);
%     plot(real_dist, 'o-'); grid on; hold on;
%     title('Real distance between node and drone');
%     xlabel('Datapoint number [-]');
%     ylabel('Distance [m]');
%     legend('Continuous, d=100m', 'Classic, d=80m');

%     % plot xy
%     figure(2);
%     plot(x, y, 'o-'); axis equal; grid on; hold on;
%     title('Measuring positions');
%     xlabel('x distance [m]');
%     ylabel('y distance[m]');
%     legend('Continuous, d=100m', 'Classic, d=80m');

    % get distance based on exp fit
    for k=1: length(fieldnames(db))
        dist_exp(k) = 0.1973*exp(-0.0902*ESP(k));
        dist_lin(k) = (ESP(k)+59.48)/(-0.1127);
        angle_est(k) = asin(z(k)/func_signal_to_distance(ESP(k),'esp'))*180/pi;
        dist_lin_corr(k) = (ESP(k)+func_attenuation_angle(angle_est(k))+52.18)/(-0.2687);
    end
    
%     % plot distance comparison
%     figure;
%     plot(real_dist, 'o-'); grid on; hold on;
%     plot(dist_exp, 'o-');
%     plot(dist_lin, 'o-');
%     plot(dist_lin_corr, 'o-');
%     title('Comparison of distances');
%     xlabel('Datapoint number [-]')
%     ylabel('Distance [m]');
%     legend('Real distance', 'Estimated distance, exponential', 'Estimated distance, linear', 'Estimated distance, corrected linear');
    
    % multilateration
    points = [x; y; zeros(size(x))];
    sol_real = Trilateration(points, real_dist, diag(ones(1,3)));
    sol_expo = Trilateration(points, dist_exp, diag(ones(1,3)));
    sol_lin  = Trilateration(points, dist_lin, diag(ones(1,3)));
    sol_linc = Trilateration(points, dist_lin_corr, diag(ones(1,3)));
    
%     % plot circles
%     figure;
%     plot(x, y, 'o'); axis equal; grid on; hold on;
%     plot(sol_real(2), sol_real(3), 'kx');
%     for k=1: length(x)
%         plot_circle(x(k), y(k), real_dist(k), 'r')
%     end
%     title('Real distances');
%     xlabel('x distance [m]');
%     ylabel('y distance[m]');
%     legend('Measuring positions', 'Solution found', 'Real distance');
    
    % plot circles
    figure;
    plot(x, y, 'o'); axis equal; grid on; hold on;
    plot(sol_expo(2), sol_expo(3), 'kx');
    for k=1: length(x)
        plot_circle(x(k), y(k), dist_exp(k), 'g')
    end
    title('Exponential fit');
    xlabel('x distance [m]');
    ylabel('y distance[m]');
    legend('Measuring positions', 'Solution found', 'Distance estimated');
    
    % plot circle group
    figure;
    plot(x, y, 'o'); axis equal; grid on; hold on;
    points_group = [mean(points(1,1:20)) mean(points(1,21:42)) mean(points(1,43:end));
                    mean(points(2,1:20)) mean(points(2,21:42)) mean(points(2,43:end));
                    zeros(1,3)];
    dist_exp_group = [mean(dist_exp(1:20)) mean(dist_exp(21:42)) mean(dist_exp(43:end))];
    sol_expo_group = Trilateration(points_group, dist_exp_group, diag(ones(1,3)));
    plot(sol_expo_group(2), sol_expo_group(3), 'kx');
    for k=1: 3
        plot_circle(points_group(1,k), points_group(2,k), dist_exp_group(k), 'g')
    end
    title('Exponential fit grouped');
    xlabel('x distance [m]');
    ylabel('y distance[m]');
    legend('Measuring positions', 'Solution found', 'Distance estimated');
    
    % plot circles
    figure;
    plot(x, y, 'o'); axis equal; grid on; hold on;
    plot(sol_lin(2), sol_lin(3), 'kx');
    for k=1: length(x)
        plot_circle(x(k), y(k), dist_lin(k), 'b')
    end
    title('Linear fit');
    xlabel('x distance [m]');
    ylabel('y distance[m]');
    legend('Measuring positions', 'Solution found', 'Distance estimated');
    
    % plot circle group
    figure;
    plot(x, y, 'o'); axis equal; grid on; hold on;
    dist_lin_group = [mean(dist_lin(1:20)) mean(dist_lin(21:42)) mean(dist_lin(43:end))];
    sol_lin_group = Trilateration(points_group, dist_lin_group, diag(ones(1,3)));
    plot(sol_lin_group(2), sol_lin_group(3), 'kx');
    for k=1: 3
        plot_circle(points_group(1,k), points_group(2,k), dist_lin_group(k), 'g')
    end
    title('Linear fit grouped');
    xlabel('x distance [m]');
    ylabel('y distance[m]');
    legend('Measuring positions', 'Solution found', 'Distance estimated');
    
    % plot circles
    figure;
    plot(x, y, 'o'); axis equal; grid on; hold on;
    plot(sol_linc(2), sol_linc(3), 'kx');
    for k=1: length(x)
        plot_circle(x(k), y(k), dist_lin_corr(k), 'y')
    end
    title('Linear fit corrected with estimated angle');
    xlabel('x distance [m]');
    ylabel('y distance[m]');
    legend('Measuring positions', 'Solution found', 'Distance estimated');
    
    % plot circle group
    figure;
    plot(x, y, 'o'); axis equal; grid on; hold on;
    dist_lin_corr_group = [mean(dist_lin_corr(1:20)) mean(dist_lin_corr(21:42)) mean(dist_lin_corr(43:end))];
    sol_lin_corr_group = Trilateration(points_group, dist_lin_corr_group, diag(ones(1,3)));
    plot(sol_lin_corr_group(2), sol_lin_corr_group(3), 'kx');
    for k=1: 3
        plot_circle(points_group(1,k), points_group(2,k), dist_lin_corr_group(k), 'y')
    end
    title('Linear fit corrected with estimated angle grouped');
    xlabel('x distance [m]');
    ylabel('y distance[m]');
    legend('Measuring positions', 'Solution found', 'Distance estimated');
    
    % print results
    fprintf('\nError obtained on file %s: \nExponential fit: %.1f meters (grouped %.1f meters)\nLinear fit: %.1f meters (grouped %.1f meters)\nCorrected linear fit: %.1f meters (grouped %.1f meters)\n', fnames(j,21:end), norm(sol_expo(2:4)), norm(sol_expo_group(2:3)), norm(sol_lin(2:4)), norm(sol_lin_group(2:3)), norm(sol_linc(2:4)), norm(sol_lin_corr_group(2:3)));
end


% plots a circle in x, y, radius r 
function plot_circle(x, y, r, color)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    plot(xunit, yunit, color);
end