% runs the simulation multiple times
clear all; close all;

% define size
number_experiments = 20;

% launch profiler
profile on;

% run the simulations
for experiment_counter=1: number_experiments
    
    % save in workspace (not good code OK, but best option...)
    save('matlab_sim_v2/temp.mat', 'experiment_counter', 'number_experiments');
    
    %check progress
    fprintf('%d... ', experiment_counter);
    
%     % for 1 drone and gradient
%     results_1drone_gradient{experiment_counter} = sim2_1drone_gradient();
%     final_precision_1drone_gradient(experiment_counter) = results_1drone_gradient{experiment_counter}.final_precision;
%     inter_precision_1drone_gradient(experiment_counter) = results_1drone_gradient{experiment_counter}.inter_precision;
%     final_time_1drone_gradient(experiment_counter) = results_1drone_gradient{experiment_counter}.final_time;
%     inter_time_1drone_gradient(experiment_counter) = results_1drone_gradient{experiment_counter}.inter_time;
    
%     % for 1 drone and continuous
%     results_1drone_continuous{experiment_counter} = sim2_1drone_continuous();
%     final_precision_1drone_continuous(experiment_counter) = results_1drone_continuous{experiment_counter}.final_precision;
%     inter_precision_1drone_continuous(experiment_counter) = results_1drone_continuous{experiment_counter}.inter_precision;
%     final_time_1drone_continuous(experiment_counter) = results_1drone_continuous{experiment_counter}.final_time;
%     inter_time_1drone_continuous(experiment_counter) = results_1drone_continuous{experiment_counter}.inter_time;
    
%     % for 1 drone and trilateration
%     results_1drone_trilateration{experiment_counter} = sim2_1drone_trilateration();
%     final_precision_1drone_trilateration(experiment_counter) = results_1drone_trilateration{experiment_counter}.final_precision;
%     inter_precision_1drone_trilateration(experiment_counter) = results_1drone_trilateration{experiment_counter}.inter_precision;
%     final_time_1drone_trilateration(experiment_counter) = results_1drone_trilateration{experiment_counter}.final_time;
%     inter_time_1drone_trilateration(experiment_counter) = results_1drone_trilateration{experiment_counter}.inter_time;
    
%     % for 1 drone and trilateration (mod)
%     results_1drone_trilateration_mod{experiment_counter} = sim2_1drone_trilateration_mod();
%     final_precision_1drone_trilateration_mod(experiment_counter) = results_1drone_trilateration_mod{experiment_counter}.final_precision;
%     inter_precision_1drone_trilateration_mod(experiment_counter) = results_1drone_trilateration_mod{experiment_counter}.inter_precision;
%     final_time_1drone_trilateration_mod(experiment_counter) = results_1drone_trilateration_mod{experiment_counter}.final_time;
%     inter_time_1drone_trilateration_mod(experiment_counter) = results_1drone_trilateration_mod{experiment_counter}.inter_time;
    
%     % for three drones and trilateration
%     results_3drone_trilateration{experiment_counter} = sim2_3drone_trilateration();
%     final_precision_3drone_trilateration(experiment_counter) = results_3drone_trilateration{experiment_counter}.final_precision;
%     inter_precision_3drone_trilateration(experiment_counter) = results_3drone_trilateration{experiment_counter}.inter_precision;
%     final_time_3drone_trilateration(experiment_counter) = results_3drone_trilateration{experiment_counter}.final_time;
%     inter_time_3drone_trilateration(experiment_counter) = results_3drone_trilateration{experiment_counter}.inter_time;
    
    % for three drones and trilateration
    results_3drone_swarm{experiment_counter} = sim2_3drone_swarm();
    final_precision_3drone_swarm(experiment_counter) = results_3drone_swarm{experiment_counter}.final_precision;
    inter_precision_3drone_swarm(experiment_counter) = results_3drone_swarm{experiment_counter}.inter_precision;
    final_time_3drone_swarm(experiment_counter) = results_3drone_swarm{experiment_counter}.final_time;
    inter_time_3drone_swarm(experiment_counter) = results_3drone_swarm{experiment_counter}.inter_time;
end
fprintf('\n');

% stop profiler
profile off;
profile viewer;

% for easier use
precision = final_precision_3drone_swarm;
time = final_time_3drone_swarm;

% plot precisions and time
figure();
plot(precision, 'bo-'); grid on; hold on;
str = sprintf('Reached precision, mean=%.1f', mean(precision));
title(str);
xlabel('Experiment');
ylabel('Precision [m]');
figure();
plot(time, 'bo-'); grid on; hold on;
str = sprintf('Final time, mean=%.1f', mean(time));
title(str);
xlabel('Experiment');
ylabel('Time [s]');


% mean_pre1 = mean(precision(1:end/2));
% mean_pre2 = mean(precision(end/2:end));
% mean_time1 = mean(time(1:end/2));
% mean_time2 = mean(time(end/2:end));


% % plot precisions
% figure();
% plot(final_precision_1drone_gradient, 'bo-'); grid on; hold on;
% plot(final_precision_1drone_trilateration, 'ro-');
% plot(final_precision_3drone_trilateration, 'go-');
% title('Reached precision');
% xlabel('Experiment');
% ylabel('Precision [m]');
% legend('Gradient', 'Trilateration1', 'Trilateration3');
% 
% % plot final time
% figure();
% plot(final_time_1drone_gradient, 'bo-'); grid on; hold on;
% plot(final_time_1drone_trilateration, 'ro-');
% plot(final_time_3drone_trilateration, 'go-');
% title('Final time');
% xlabel('Experiment');
% ylabel('Time [s]');
% legend('Gradient', 'Trilateration1', 'Trilateration3');

delete('matlab_sim_v2\temp.mat');
