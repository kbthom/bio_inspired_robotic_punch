function output_data = Experiment_Example_MATLAB()
    figure(1);  clf;       % Create an empty figure to update later
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Position (rad)');
    title('Position');
    

    function my_callback(new_data)
        t = new_data(:,1);   % time
        pos = new_data(:,2); % position
        N = length(pos);
        
        
        h1.XData(end+1:end+N) = t;   % Update subplot 1
        h1.YData(end+1:end+N) = pos;


    end
    
    frdm_ip  = '192.168.1.100';     % Nucleo board ip
    frdm_port= 11223;               % Nucleo board port  
    params.callback = @my_callback; % callback function
    params.timeout  = 2;            % end of experiment timeout
    
    % The example program provided takes two arguments
    Kp = 0.0;
    Ki = 0.0;
    Desired_Current = 1.0;
    input = [Kp Ki Desired_Current];    % input sent to Nucleo board
    output_size = 5;    % number of outputs expected
    
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
        
    
end
