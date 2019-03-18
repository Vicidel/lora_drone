% runs the simulation multiple times
clear all; close all;

% define size
number_runs = 50;

% run the simulations
for k=1: number_runs
    results{k} = sim2_1drone_gradient();
    final_precision(k) = results{k}.final_precision;
    final_time(k) = results{k}.time_final;
end

% plot precisions
figure();
plot(final_precision, 'o-'); grid on;
title('Reached precision');
xlabel('Experiment');
ylabel('Precision [m]');

% plot final time
figure();
plot(final_time, 'o-'); grid on;
title('Final time');
xlabel('Experiment');
ylabel('Time [s]');
