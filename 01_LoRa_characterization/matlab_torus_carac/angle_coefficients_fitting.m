% create data
angles = [0, 15, 30, 45, 60]';
attenuation = [0 -5.5 -15.1 -22.1 -34.2]' - 3;
attenuation = - attenuation;

% prepare data
[xData, yData] = prepareCurveData( angles, attenuation );

% Set up fittype and options.
ft = fittype( 'a*x+b', 'independent', 'x', 'dependent', 'y' );
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