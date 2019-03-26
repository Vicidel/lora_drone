% runs the simulation multiple times
clear all; close all;

% define size
number_experiments = 100;

% % launch profiler
% profile on;

% simulations to test
test_1drone_gradient          = 0;
test_1drone_continuous        = 0;
test_1drone_trilateration     = 0;
test_1drone_trilateration_mod = 0;
test_3drone_continuous        = 1;
test_3drone_trilateration     = 0;
test_3drone_swarm             = 0;

% run the simulations
for experiment_counter=1: number_experiments
    
    % save in workspace (not good code OK, but best option...)
    save('matlab_sim_v2/temp.mat', 'experiment_counter', 'number_experiments');
    
    %check progress
    fprintf('Run number %d...', experiment_counter);
    tic
    
    if test_1drone_gradient
        % for 1 drone and gradient
        results_1drone_gradient{experiment_counter} = sim2_1drone_gradient();
        final_precision_1drone_gradient(experiment_counter) = results_1drone_gradient{experiment_counter}.final_precision;
        inter_precision_1drone_gradient(experiment_counter) = results_1drone_gradient{experiment_counter}.inter_precision;
        final_time_1drone_gradient(experiment_counter) = results_1drone_gradient{experiment_counter}.final_time;
        inter_time_1drone_gradient(experiment_counter) = results_1drone_gradient{experiment_counter}.inter_time;
    else
        final_precision_1drone_gradient(experiment_counter) = 0;
        inter_precision_1drone_gradient(experiment_counter) = 0;
        final_time_1drone_gradient(experiment_counter) = 0;
        inter_time_1drone_gradient(experiment_counter) = 0;
    end
    
    if test_1drone_continuous
        % for 1 drone and continuous
        results_1drone_continuous{experiment_counter} = sim2_1drone_continuous();
        final_precision_1drone_continuous(experiment_counter) = results_1drone_continuous{experiment_counter}.final_precision;
        inter_precision_1drone_continuous(experiment_counter) = results_1drone_continuous{experiment_counter}.inter_precision;
        final_time_1drone_continuous(experiment_counter) = results_1drone_continuous{experiment_counter}.final_time;
        inter_time_1drone_continuous(experiment_counter) = results_1drone_continuous{experiment_counter}.inter_time;
    else
        final_precision_1drone_continuous(experiment_counter) = 0;
        inter_precision_1drone_continuous(experiment_counter) = 0;
        final_time_1drone_continuous(experiment_counter) = 0;
        inter_time_1drone_continuous(experiment_counter) = 0;
    end
    
    if test_1drone_trilateration
        % for 1 drone and trilateration
        results_1drone_trilateration{experiment_counter} = sim2_1drone_trilateration();
        final_precision_1drone_trilateration(experiment_counter) = results_1drone_trilateration{experiment_counter}.final_precision;
        inter_precision_1drone_trilateration(experiment_counter) = results_1drone_trilateration{experiment_counter}.inter_precision;
        final_time_1drone_trilateration(experiment_counter) = results_1drone_trilateration{experiment_counter}.final_time;
        inter_time_1drone_trilateration(experiment_counter) = results_1drone_trilateration{experiment_counter}.inter_time;
    else
        final_precision_1drone_trilateration(experiment_counter) = 0;
        inter_precision_1drone_trilateration(experiment_counter) = 0;
        final_time_1drone_trilateration(experiment_counter) = 0;
        inter_time_1drone_trilateration(experiment_counter) = 0;
    end
    
    if test_1drone_trilateration_mod
        % for 1 drone and trilateration (mod)
        results_1drone_trilateration_mod{experiment_counter} = sim2_1drone_trilateration_mod();
        final_precision_1drone_trilateration_mod(experiment_counter) = results_1drone_trilateration_mod{experiment_counter}.final_precision;
        inter_precision_1drone_trilateration_mod(experiment_counter) = results_1drone_trilateration_mod{experiment_counter}.inter_precision;
        final_time_1drone_trilateration_mod(experiment_counter) = results_1drone_trilateration_mod{experiment_counter}.final_time;
        inter_time_1drone_trilateration_mod(experiment_counter) = results_1drone_trilateration_mod{experiment_counter}.inter_time;
    else
        final_precision_1drone_trilateration_mod(experiment_counter) = 0;
        inter_precision_1drone_trilateration_mod(experiment_counter) = 0;
        final_time_1drone_trilateration_mod(experiment_counter) = 0;
        inter_time_1drone_trilateration_mod(experiment_counter) = 0;
    end    
    
    if test_3drone_continuous
        %for 3 drone and continuous
        results_3drone_continuous{experiment_counter} = sim2_3drone_continuous();
        final_precision_3drone_continuous(experiment_counter) = results_3drone_continuous{experiment_counter}.final_precision;
        inter_precision_3drone_continuous(experiment_counter) = results_3drone_continuous{experiment_counter}.inter_precision;
        final_time_3drone_continuous(experiment_counter) = results_3drone_continuous{experiment_counter}.final_time;
        inter_time_3drone_continuous(experiment_counter) = results_3drone_continuous{experiment_counter}.inter_time;
    else
        final_precision_3drone_continuous(experiment_counter) = 0;
        inter_precision_3drone_continuous(experiment_counter) = 0;
        final_time_3drone_continuous(experiment_counter) = 0;
        inter_time_3drone_continuous(experiment_counter) = 0;
    end
    
    if test_3drone_trilateration
        % for three drones and trilateration
        results_3drone_trilateration{experiment_counter} = sim2_3drone_trilateration();
        final_precision_3drone_trilateration(experiment_counter) = results_3drone_trilateration{experiment_counter}.final_precision;
        inter_precision_3drone_trilateration(experiment_counter) = results_3drone_trilateration{experiment_counter}.inter_precision;
        final_time_3drone_trilateration(experiment_counter) = results_3drone_trilateration{experiment_counter}.final_time;
        inter_time_3drone_trilateration(experiment_counter) = results_3drone_trilateration{experiment_counter}.inter_time;
    else
        final_precision_3drone_trilateration(experiment_counter) = 0;
        inter_precision_3drone_trilateration(experiment_counter) = 0;
        final_time_3drone_trilateration(experiment_counter) = 0;
        inter_time_3drone_trilateration(experiment_counter) = 0;
    end
    
    if test_3drone_swarm
        % for three drones and swarming
        results_3drone_swarm{experiment_counter} = sim2_3drone_swarm();
        final_precision_3drone_swarm(experiment_counter) = results_3drone_swarm{experiment_counter}.final_precision;
        inter_precision_3drone_swarm(experiment_counter) = results_3drone_swarm{experiment_counter}.inter_precision;
        final_time_3drone_swarm(experiment_counter) = results_3drone_swarm{experiment_counter}.final_time;
        inter_time_3drone_swarm(experiment_counter) = results_3drone_swarm{experiment_counter}.inter_time;
    else
        final_precision_3drone_swarm(experiment_counter) = 0;
        inter_precision_3drone_swarm(experiment_counter) = 0;
        final_time_3drone_swarm(experiment_counter) = 0;
        inter_time_3drone_swarm(experiment_counter) = 0;
    end
    
    % store execution time
    exec_time(experiment_counter) = toc;
    fprintf(' done in %.2f seconds\n', exec_time(experiment_counter)); 
end
fprintf('\n');

% % stop profiler
% profile off;
% profile viewer;

% nanmean(final_precision_1drone_trilateration)
% nanmean(final_precision_1drone_trilateration(1:end/2))
% nanmean(final_precision_1drone_trilateration(end/2:end))
% nanmean(final_time_1drone_trilateration)
% nanmean(final_time_1drone_trilateration(1:end/2))
% nanmean(final_time_1drone_trilateration(end/2:end))

% plot final precisions
figure(); grid on; hold on;
% plot(final_precision_1drone_gradient, 'r-'); 
% plot(final_precision_1drone_continuous, 'g-'); 
% plot(final_precision_1drone_trilateration, 'b-');
% plot(final_precision_1drone_trilateration_mod, 'co-'); 
plot(final_precision_3drone_continuous, 'mo-'); 
% plot(final_precision_3drone_trilateration, 'yo-'); 
% plot(final_precision_3drone_swarm, 'ko-'); 
% legend('One drone, gradient', 'One drone, continuous', 'One drone, trilateration', ...
%         'One drone, trilateration mod', 'Three drone, continuous', 'Three drone, trilateration', 'Three drone, continuous');
% legend('Gradient descent', 'Multilateration', 'Trilateration');
title('Final precision');
xlabel('Experiment');
ylabel('Precision [m]');

% plot final times
figure(); grid on; hold on;
% plot(final_time_1drone_gradient, 'r-'); 
% plot(final_time_1drone_continuous, 'g-'); 
% plot(final_time_1drone_trilateration, 'b-');
% plot(final_time_1drone_trilateration_mod, 'co-'); 
plot(final_time_3drone_continuous, 'mo-'); 
% plot(final_time_3drone_trilateration, 'yo-'); 
% plot(final_time_3drone_swarm, 'ko-');
% legend('One drone, gradient', 'One drone, continuous', 'One drone, trilateration', ...
%         'One drone, trilateration mod', 'Three drone, continuous', 'Three drone, trilateration', 'Three drone, swarming');
% legend('Gradient descent', 'Multilateration', 'Trilateration');
title('Final time');
xlabel('Experiment');
ylabel('Time [s]');

% plot inter
plot_inter_bool = false;
if plot_inter_bool
    % plot inter precisions
    figure();
    plot(inter_precision_1drone_gradient, 'ro-'); grid on; hold on;
    plot(inter_precision_1drone_continuous, 'go-'); 
    plot(inter_precision_1drone_trilateration, 'bo-'); 
    plot(inter_precision_1drone_trilateration_mod, 'co-'); 
    plot(inter_precision_3drone_continuous, 'mo-'); 
    plot(inter_precision_3drone_trilateration, 'yo-'); 
    plot(inter_precision_3drone_swarm, 'ko-'); 
    legend('One drone, gradient', 'One drone, continuous', 'One drone, trilateration', ...
            'One drone, trilateration mod', 'Three drone, continuous', 'Three drone, trilateration', 'Three drone, swarming');
    title('Intermediary precision');
    xlabel('Experiment');
    ylabel('Precision [m]');

    % plot times
    figure();
    plot(inter_time_1drone_gradient, 'ro-'); grid on; hold on;
    plot(inter_time_1drone_continuous, 'go-'); 
    plot(inter_time_1drone_trilateration, 'bo-'); 
    plot(inter_time_1drone_trilateration_mod, 'co-'); 
    plot(inter_time_3drone_continuous, 'mo-'); 
    plot(inter_time_3drone_trilateration, 'yo-'); 
    plot(inter_time_3drone_swarm, 'ko-');
    legend('One drone, gradient', 'One drone, continuous', 'One drone, trilateration', ...
            'One drone, trilateration mod', 'Three drone, continuous', 'Three drone, trilateration', 'Three drone, continuous');
    title('Intermediary time');
    xlabel('Experiment');
    ylabel('Time [s]');
end

% compute means
means_final_precision = [mean(final_precision_1drone_gradient), mean(final_precision_1drone_continuous), ...
                         mean(final_precision_1drone_trilateration), mean(final_precision_1drone_trilateration_mod), ...
                         mean(final_precision_3drone_continuous), mean(final_precision_3drone_trilateration), ...
                         mean(final_precision_3drone_swarm)];
means_final_time      = [mean(final_time_1drone_gradient), mean(final_time_1drone_continuous), ...
                         mean(final_time_1drone_trilateration), mean(final_time_1drone_trilateration_mod), ...
                         mean(final_time_3drone_continuous), mean(final_time_3drone_trilateration), ...
                         mean(final_time_3drone_swarm)];
means_inter_precision = [mean(inter_precision_1drone_gradient), mean(inter_precision_1drone_continuous), ...
                         mean(inter_precision_1drone_trilateration), mean(inter_precision_1drone_trilateration_mod), ...
                         mean(inter_precision_3drone_continuous), mean(inter_precision_3drone_trilateration), ...
                         mean(inter_precision_3drone_swarm)];
means_inter_time      = [mean(inter_time_1drone_gradient), mean(inter_time_1drone_continuous), ...
                         mean(inter_time_1drone_trilateration), mean(inter_time_1drone_trilateration_mod), ...
                         mean(inter_time_3drone_continuous), mean(inter_time_3drone_trilateration), ...
                         mean(inter_time_3drone_swarm)];

% delete files
delete('matlab_sim_v2\temp.mat');
