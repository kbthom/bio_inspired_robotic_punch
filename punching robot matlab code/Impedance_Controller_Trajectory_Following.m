% This is the main MATLAB script for Lab 5.
%
% You will need to modify the Mbed code and this script, but should not need to make any other changes.
%
%% SET YOUR INPUTS HERE

% Bezier curve control points
const_point = [.2; 0]; %[x;y] or [q1,q2] constant coordinate (x,q1,q2 coordinates should be opposite sign due to direction motors are mounted)
pts_foot = repmat(const_point,1,8);

%%pts_foot = [    -0.0935   -0.0935   -0.0935   -0.0129    0.0759    0.1507    0.2313    0.2313    0.2313;
  %% -0.1359   -0.1359   -0.1359   -0.1686   -0.1300   -0.0810   -0.0412   -0.0412   -0.0412]; % YOUR BEZIER PTS HERE

  pts_foot = [-0.0989  -0.0989  -0.0989  -0.0332  0.0573  0.1376  0.1931  0.1931  0.1931;  -0.1332  -0.1332  -0.1332  -0.1639  -0.1624  -0.0996  -0.0252  -0.0252  -0.02]; % YOUR BEZIER PTS HERE
        
% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = 0;
angle2_init = -pi/2; 

% Total experiment time is buffer,trajectory,buffer
traj_time         = .2;
pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 2;

% Gains for impedance controller
% If a gain is not being used in your Mbed code, set it to zero
% For joint space control, use K_xx for K1, K_yy for K2, D_xx for D1, D_yy for D2
gains.K_xx = 1000;
gains.K_yy = 1000;
gains.K_xy = 0;

gains.D_xx = 10;
gains.D_yy = 10;
gains.D_xy = 0;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 1;

%% Run Experiment
[output_data] = Experiment_trajectory( angle1_init, angle2_init, pts_foot,...
                                       traj_time, pre_buffer_time, post_buffer_time,...
                                       gains, duty_max);

%% Extract data
t = output_data(:,1);
x = -output_data(:,12); % actual foot position in X (negative due to direction motors are mounted)
y = output_data(:,13); % actual foot position in Y
   
xdes = -output_data(:,16); % desired foot position in X (negative due to direction motors are mounted)
ydes = output_data(:,17); % desired foot position in Y

%% Plot foot vs desired
figure(3); clf;
subplot(211); hold on
plot(t,xdes,'r-'); plot(t,x);
xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired','Actual'});

subplot(212); hold on
plot(t,ydes,'r-'); plot(t,y);
xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired','Actual'});

figure(4); clf; hold on
plot(xdes,ydes,'r-'); plot(x,y,'k');
xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired','Actual'});
