% runs the simulation multiple times
clear all; close all;

% define size
number_runs = 20;

% launch profiler
profile on;

% run the simulations
for k=1: number_runs
    %check progress
    fprintf('%d... ', k);
    
%     % for 1 drone and gradient
%     results_1drone_gradient{k} = sim2_1drone_gradient();
%     final_precision_1drone_gradient(k) = results_1drone_gradient{k}.final_precision;
%     final_time_1drone_gradient(k) = results_1drone_gradient{k}.final_time;
    
%     % for 1 drone and trilateration
%     results_1drone_trilateration{k} = sim2_1drone_trilateration();
%     final_precision_1drone_trilateration(k) = results_1drone_trilateration{k}.final_precision;
%     final_time_1drone_trilateration(k) = results_1drone_trilateration{k}.final_time;
    
%     % for 1 drone and trilateration (mod)
%     results_1drone_trilateration_mod{k} = sim2_1drone_trilateration_mod();
%     final_precision_1drone_trilateration_mod(k) = results_1drone_trilateration_mod{k}.final_precision;
%     final_time_1drone_trilateration_mod(k) = results_1drone_trilateration_mod{k}.final_time;
    
%     % for three drones and trilateration
%     results_3drone_trilateration{k} = sim2_3drone_trilateration();
%     final_precision_3drone_trilateration(k) = results_3drone_trilateration{k}.final_precision;
%     final_time_3drone_trilateration(k) = results_3drone_trilateration{k}.final_time;
end
fprintf('\n');

% stop profiler
profile off;
profile viewer;

% % plot precisions and time
% figure();
% plot(final_precision_1drone_trilateration, 'bo-'); grid on; hold on;
% plot(final_precision_1drone_trilateration_mod, 'ro-');
% figure();
% plot(final_time_1drone_trilateration, 'bo-'); grid on; hold on;
% plot(final_time_1drone_trilateration_mod, 'ro-');
% 
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


