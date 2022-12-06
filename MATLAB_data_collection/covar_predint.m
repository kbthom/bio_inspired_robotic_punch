x = x_vals
y = all_data

% Make x and y inputs into columns
x=reshape(x, [], 1); 
y=reshape(y, [], 1);

% Here starts the "guts" of the program, which you should run on your own
% data

% Define the functional form of the fit
fit_type = fittype('a*exp(b*x)');

% Execute the fit
% You might need to change the start point or add advanced fit options
[fit_object, GoF, output] = fit(x, y, fit_type, 'StartPoint', [1, 1]); 

% Unpack curve fit parameters
params = coeffvalues(fit_object);

% Compute the confidence interval for the fit parameters
param_CI = confint(fit_object, 0.95);

% For a symmetric confidence interval, the uncertainty of the parameter 
% is half of the width of the confidence interval
param_uncertainty = 0.5*diff(param_CI);

J = output.Jacobian;    % Jacobian matrix from fit process
res = output.residuals; % residual = observation (data) - fit
N = numel(x);           % Number of observations
p = numel(params);      % Number of fit parameters

MSE = (res'*res)/(N-p);               % Mean square residual matrix
% cov_matrix = inv(J'*J) * MSE;       % Covariance matrix, for converged solution
% For numerical stability, calculate cov_matrix in a slightly different way
% Ref: https://www.mathworks.com/help/releases/R2008a/toolbox/optim/ug/f19096.html#bre4quc-1
[Q,R] = qr(J,0);
inv_R = inv(R);
JTJ_inv = inv_R * (inv_R');
cov_matrix = JTJ_inv * MSE;
coeff_std_errors = sqrt(diag(cov_matrix));

%  Alternative method to calculate parameter uncertainty
% param_unc = t_factor * coeff_std_errors; 

% For our quadratic model:
a = params(1);
b = params(2);
%c = params(3);

 % For this example we expect max at x = -b/2a:
x_max = -b/(2*a)     
y_fit_at_x_max = feval(fit_object, x_max)

% For our quadratic model:
dx_da = b/(2*a^2);
dx_db = -1.0/(2*a);
SE_a2 = cov_matrix(1, 1);
SE_b2 = cov_matrix(2, 2);
SE_ab = cov_matrix(1, 2);

% Propagation of uncertainty
% Covariance term SE_ab = 0 if a and b are independent
% For least squares curve fit, fit params are usually not independent
SE_x2 = SE_a2*(dx_da)^2 + SE_b2*(dx_db)^2 + 2*SE_ab*(dx_da)*(dx_db);
SE_x = sqrt(SE_x2);

conf_level = 0.95;  % 2.671 convention is 95% confidence
% Find t-factor for 2-tail, with nu = N-p degrees of freedom
t_factor = tinv(1-(1-conf_level)/2., N-p);      
x_unc_at_x_max = t_factor * SE_x   % Convert standard error to 95% unc.

% New x variable over the domain
x_ = linspace(min(x), max(x), 100);

% Construct prediction interval of the function
y_predint = predint(fit_object, x_, conf_level, 'functional', 'off');

% Calculate confidence interval of the prediction value at a 
% specific x value
% x_calc = 10.5;    % the specific x value of interest 

% Can be a vector of values for your own data using the code below:
% x_calc = [5, 10, 15];
% x_calc = reshape(x_calc, [], 1); % Make x_calc a column

% fit value of y at this x value
% y_fit_at_x_calc = feval(fit_object, x_calc);

% Calculate confidence interval of the prediction value at this x value
% y_CI_at_x_calc = predint(fit_object, x_calc, conf_level, 'functional', 'off');

% Convert confidence interval of prediction value to uncertainty (divide by 2)
% y_unc_at_x_calc = (y_CI_at_x_calc(:, 2) - y_CI_at_x_calc(:, 1))/2.;

% Calculate confidence interval of the maximum value found above

% Calculate confidence interval of the prediction value at the max location
y_CI_at_x_max = predint(fit_object, x_max, conf_level, 'functional', 'off');

% Convert confidence interval of prediction value to uncertainty
y_unc_at_x_max = (y_CI_at_x_max(:, 2) - y_CI_at_x_max(:, 1))/2.

% Plot results
figure(1);
hold off;
plot(x, y, 'ro');  % Plot data
xlabel('Sugar Levels(Grams)');
ylabel('CO_2 Concentration (%)');
title("CO_2 Concentration at Different Sugar Levels")
hold on;
% Plot the least squares fit of the data
plot(x_, feval(fit_object, x_), 'b-');  
% Plot the 95% confidence prediction interval
plot(x_, y_predint, 'g--');   
% Plot the value withveritcal error bar at the specific x
% Plot the vertical error bar on the predicted value at specific x
% he1 = errorbar(x_calc, y_fit_at_x_calc, y_unc_at_x_calc, 'vertical');
% he1.Color = 'k';
% % Plot the value at specific x. Do this last so it appears on top of (in 
% % front of) the error bars
% plot(x_calc, y_fit_at_x_calc, 'ks');

% Plot the vertical error bar on the predicted max value
% This is the uncertainty of the estimated maximum value
he1 = errorbar(x_max, y_fit_at_x_max, y_unc_at_x_max, 'vertical');
% Plot the horizontal error bar on the predicted max value
% This is the uncertainty fo the location of the max value
he2 = errorbar(x_max, y_fit_at_x_max, x_unc_at_x_max, 'horizontal');
% Max the error bars the same colour as the prediction interval 
he1.Color = 'k';
he2.Color='k';
% Plot the estimated max value. Do this last so it appears on top of (in 
% front of) the error bars
plot(x_max, y_fit_at_x_max, 'ks');
improvePlot;
ax = gca;
ax.YRuler.Exponent = 0
ax.TickDir = 'out';
ac.FontSiza = 12;
