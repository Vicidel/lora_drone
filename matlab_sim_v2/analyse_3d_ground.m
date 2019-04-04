% requires a dataset named data 
data = final_precision_1drone_trilateration;

nb = length(data);

data_10  = data(1:nb/4)';
data_20  = data(1+nb/4:nb/2)';
data_50  = data(1+nb/2:3*nb/4)';
data_100 = data(1+3*nb/4:nb)';

% plot
figure(); grid on; hold on;
group = [ones(size(data_10)); 2*ones(size(data_20)); 3*ones(size(data_50)); 4*ones(size(data_100))];
boxplot([data_10, data_20, data_50, data_100], group);
set(gca,'XTickLabel',{'20m, base', '20m, mod', '100m, base', '100m, mod'});
xlabel('Experiment set');
ylabel('Precision reached [m]');
title('Precision based on experiment set');
