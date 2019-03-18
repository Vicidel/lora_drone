% runs the simulation multiple times
clear all; close all;

% define size
number_runs = 3;

% run the simulations
for k=1: number_runs
    % check progress
    fprintf('%d... ', k);
    
    % for 1 drone and gradient
    results_1drone_gradient{k} = sim2_1drone_gradient();
    final_precision_1drone_gradient(k) = results_1drone_gradient{k}.final_precision;
    final_time_1drone_gradient(k) = results_1drone_gradient{k}.time_final;
    
    % for 1 drone and trilateration
    results_1drone_trilateration{k} = sim2_1drone_trilateration();
    final_precision_1drone_trilateration(k) = results_1drone_trilateration{k}.final_precision;
    final_time_1drone_trilateration(k) = results_1drone_trilateration{k}.time_final;
end

% plot precisions
figure();
plot(final_precision_1drone_gradient, 'bo-'); grid on; hold on;
plot(final_precision_1drone_trilateration, 'ro-');
title('Reached precision');
xlabel('Experiment');
ylabel('Precision [m]');
legend('Gradient', 'Trilateration');

% plot final time
figure();
plot(final_time_1drone_gradient, 'bo-'); grid on; hold on;
plot(final_time_1drone_trilateration, 'ro-');
title('Final time');
xlabel('Experiment');
ylabel('Time [s]');
legend('Gradient', 'Trilateration');
