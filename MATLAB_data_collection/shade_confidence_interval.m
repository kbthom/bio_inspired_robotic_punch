function shade_confidence = shade_confidence_interval(x, y)

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
    
    conf_level = 0.95;  % 2.671 convention is 95% confidence
    
    % New x variable over the domain
    x_ = linspace(min(x), max(x), 100);
    
    % Construct prediction interval of the function
    y_predint = predint(fit_object, x_, conf_level, 'functional', 'off');
    
    
    % Plot results
    figure(1);
    hold off;
    
    % Do shading first so it appears on the bottom
      
    % Define the upper bound of the shaded area
    y_upper = y_predint(:, 2);
    
    % Define the lower bound of the shaded area 
    y_lower = y_predint(:, 1);
    
    % Setup the arrays for shading a closed region
    x2 = [x_, fliplr(x_)];
    in_between = [y_upper; flipud(y_lower)];
    
    % Shade the region
    hf = fill(x2, in_between, 'm', 'DisplayName', '95% confidence bounds');
    hold on;
    
    % Adjust some of the parameters of the shading
    hf.FaceAlpha = 0.5; % Make shading semi-transparent
    hf.FaceColor = 'm'; % Set color of shaded region
    hf.EdgeColor = 'none'; % Set color of edge of shaded region
    
    % Plot the least squares fit of the data
    plot(x_, feval(fit_object, x_), 'b-', 'DisplayName', 'LSQ fit');  
    
    plot(x, y, 'ro', 'DisplayName', 'Data');  % Plot data last so it appears on top
    xlabel('Number of weights on Upper Arm');
    ylabel('Momentum (Newton seconds)');
    legend;
    improvePlot;
end