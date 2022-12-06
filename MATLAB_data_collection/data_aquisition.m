all_data = []
x_vals = []
for u = 0:6

    [trial_data, num] = get_data(u,6-u);
    x_array = ones(1, num)*u;
    all_data  = [all_data trial_data];
    x_vals = [x_vals x_array]
end
all_data;


function [max_force_array, num] = get_data(upper_val, lower_val)
    upper_val = int2str(upper_val);
    lower_val = int2str(lower_val);
    folder_size=dir(['data/new/upper', upper_val, '_lower', lower_val, '/Good /*.csv']);
    out=size(folder_size,1);
    
    for y = 1:out
        folder = strcat('data/new/upper', upper_val, '_lower', lower_val, '/Good/upper', upper_val, '_lower', lower_val, '_trial', int2str(y), '.csv');
        
        [header, data] = load_CSV_file(folder);
        d_range = 12:length(data)-2;
        offset_range = 12:1000;
        time_data = data(d_range, 1) - data(12, 1);
        avg_offset = mean(data(offset_range, 2));
        
        force_data = data(d_range, 2)- avg_offset;
        %figure(1)
        %plot(time_data, force_data);
        
        %improvePlotnew()
        %ylabel('Force(N)')
        %xlabel('Time(sec)')
        %title('Force vs Time')
        %avg_offset = mean(force_data);
        max_force = max(force_data);
        max_force_array(y) = max_force;
    end
    num = out;
    
    
    function momentum = momentum_calc(force_data, avg_offset)
    
        time_counter = 0;
        [max_val, index] = max(force_data);
    
        for i = index:length(force_data)
            if force_data(i) > 2
                time_counter = time_counter +1;
                continue
            else
                break   
            end
        end
    
        punch_data1 = force_data(index: i);
        
        for j = index:-1:2
            if abs(force_data(j)) > 2
                time_counter = time_counter +1;
                continue
            else
                break   
            end
        end
    
    
        punch_data2 = force_data(j:index-1);
    
        
        punch_data_true = [punch_data2',  punch_data1'];
    
       time_interval = time_counter/1000;
       tspan = linspace(0, time_interval,length(punch_data_true));
       momentum = 0;
       
       for k = 1:length(punch_data_true)
           momentum = momentum + punch_data_true(k)*time_interval;
       end
       %punch_data = punch_data1;
        %figure(3)
        %plot(tspan,punch_data_true)
        %title(momentum)
        %}
    end
end