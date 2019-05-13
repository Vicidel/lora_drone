% create data
angles = [0, 15, 30, 45, 60, 75, 90]';
attenuation = [4, 2, 1, -1, -5, -10, -20]' - 3;
attenuation = - attenuation;

% prepare data
[xData, yData] = prepareCurveData( angles, attenuation );

% Set up fittype and options.
ft = fittype( 'exp1' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [0.182619786112888 0.0561273032068501];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
figure();
h = plot( fitresult, xData, yData );
legend( h, 'attenuation vs. angles', 'untitled fit 1', 'Location', 'NorthEast' );
% Label axes
xlabel angles
ylabel attenuation
grid on