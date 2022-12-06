[header, data] = load_CSV_file();
d_range = 12:length(data)-2;
offset_range = 12:1000;
time_data = data(d_range, 1) - data(12, 1);
avg_offset = mean(data(offset_range, 2));

force_data = data(d_range, 2)- avg_offset;
figure(1)
plot(time_data, force_data);

improvePlotnew()
ylabel('Force(N)')
xlabel('Time(sec)')
title('Force vs Time')
avg_offset = mean(force_data);
[momentum, punch_data] = momentum_calc(force_data, avg_offset)


function [momentum, punch_data] = momentum_calc(force_data, avg_offset)
    epsilon = 1.6;

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

    punch_data1 = force_data(index: i)
    
    for j = index:-1:2
        if abs(force_data(j)) > 2
            time_counter = time_counter +1;
            continue
        else
            break   
        end
    end

    momentum = punch_data1;
    punch_data2 = force_data(j:index-1);
    punch_data = punch_data2;

    
    punch_data_true = [punch_data2',  punch_data1'];

   time_interval = time_counter/1000;
   tspan = linspace(0, time_interval,length(punch_data_true));
   momentum = 0;
   
   for k = 1:length(punch_data_true)
       momentum = momentum + punch_data_true(k)*time_interval;
   end
   %punch_data = punch_data1;
    figure(3)
    plot(tspan,punch_data_true)
    %}
end
