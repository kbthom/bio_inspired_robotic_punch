%% PARAMETER SWEEP 
tot_m = 0.12;
ratio_list = [.1 .3 .6 ];
ratio_step = 0.02/0.12; %5 gram resolution
%ratio_step = 0.4;
ratio_list = [0:ratio_step:1];
peaks = [];
m2_over_m4 = [];
xmasses = [];
xvels = []; 
peakmasses = [];
peakxvels = [];


animate=false;
trial_figures=false;
for i = 1:length(ratio_list)
    m2_ratio = ratio_list(i);
    m4_ratio = 1 - m2_ratio;
    output = simulate_arm(m2_ratio, m4_ratio,tot_m,animate,trial_figures);
    peak = output(1);
    peak_idx = output(2);
    xmass = output(3);
    xvel = output(4);
    peakmass = output(5);
    peakxvel = output(6);

    m2_over_m4(i) = (m2_ratio*tot_m+.0304)/(m4_ratio*tot_m+.0189);
    peaks(i) = peak;
    xmasses(i) = xmass;
    xvels(i) = xvel;
    peakmasses(i) = peakmass;
    peakxvels(i) = peakxvel;
end

figure(20)
plot(ratio_list,xmasses,'k','LineWidth',2)
title 'X mass at plate'
xlabel 'Percent of Added Mass to M2'
ylabel 'X mass'

figure(21)
plot(ratio_list,xvels,'k','LineWidth',2)
title 'X Vel at plate'
xlabel 'Percent of Added Mass to M2'
ylabel 'X Velocity'

figure(22)
plot(ratio_list,peaks,'k','LineWidth',2)
title 'Momentum of Punch at plate'
xlabel 'Percent of Added Mass to Upper Arm'
ylabel 'X Momentum'

figure(25)
plot(m2_over_m4,peaks,'k','LineWidth',2)
title 'Momentum of Punch at plate'
xlabel 'M2 / M4'
ylabel 'X Momentum [kg*m/s]'


figure(23)
plot(ratio_list,peakmasses,'k','LineWidth',2)
title 'Peak X mass'
xlabel 'Percent of Added Mass to M2'
ylabel 'Peak X Mass'

figure(24)
plot(ratio_list,peakxvels,'k','LineWidth',2)
title 'Peak X velo'
xlabel 'Percent of Added Mass to M2'
ylabel 'Peak X Velo'

% figure
% plot(m2_over_m4,xmasses,'k','LineWidth',2)
% title 'X mass at peak momentum'
% xlabel 'M2 / M4'
% ylabel 'X mass at peak'
% 
% figure
% plot(m2_over_m4,xvels,'k','LineWidth',2)
% title 'X Vel at peak momentum'
% xlabel 'M2 / M4'
% ylabel 'X Velocity at Peak'
% 
% figure
% plot(m2_over_m4,peaks,'k','LineWidth',2)
% title 'Parameter Sweep'
% xlabel 'M2 / M4'
% ylabel 'Peak X Momentum'

% figure
% plot(m2_over_m4,peakmasses,'k','LineWidth',2)
% title 'Peak X mass'
% xlabel 'm2 /m4'
% ylabel 'Peak X Mass'
% 
% figure
% plot(m2_over_m4,peakxvels,'k','LineWidth',2)
% title 'Peak X velo'
% xlabel 'm2 / m4'
% ylabel 'Peak X Velo'



function output = simulate_arm(m2_ratio, m4_ratio,tot_m,animate,trial_figures)
    addpath('auto_derived\')
    addpath('animate\')
    addpath('modeling\')
    %% Define fixed paramters
    m1 =0.0238;             m2 =.0304; 
    m3 = .0037;             m4 = .0189;
    I1 = 11584.75 * 10^-9;      I2 = 27498.98 * 10^-9;
    I3 = 6668.66 * 10^-9;      I4 = 23804.22 * 10^-9;
    l_OA=.011;              l_OB=.042; 
    l_AC=.096;              l_DE=.091;
    l_O_m1=0.02606;           l_B_m2=0.04415; 
    l_A_m3=0.048;          l_C_m4=0.0283234;
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81;    
    r_masses=0.025;

    % Distribute Mass amongst the linkages
    m2 =m2+m2_ratio*tot_m; 
    m4 = m4+m4_ratio*tot_m;

    I2 = I2+0.5*m2_ratio*tot_m*r_masses^2;
    I4 = I4+0.5*m4_ratio*tot_m*r_masses^2;
    %% Parameter vector
    p   = [m1 m2 m3 m4 I1 I2 I3 I4 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_OA l_OB l_AC l_DE g]';
    
    %% Perform Dynamic simulation
    dt = 0.001;
    tf = 5;
    num_step = floor(tf/dt);
    tspan = linspace(0, tf, num_step); 
    z0 = [0; pi/2; 0; 0];
    z_out = zeros(4,num_step);
    z_out(:,1) = z0;
    tau_out = zeros(2,num_step);

    pts_foot = [-0.0989  -0.0989  -0.0989  -0.0332  0.0573  0.1376  0.1931  0.1931  0.1931;
        -0.1332  -0.1332  -0.1332  -0.1639  -0.1624  -0.0996  -0.0252  -0.0252  -0.02]; % YOUR BEZIER PTS HERE
    t_traj = 0.2;
    
    num_steps = floor(t_traj/dt);
    ctrl_t = linspace(0, t_traj, num_steps); 
    n = length(ctrl_t);
    rEd = zeros(2,n);

    for i=1:n
    rEd(:,i) = BezierCurve(pts_foot,ctrl_t(i)/t_traj);
    end

    targets = zeros(2,length(tspan));
    targetv  =zeros(2,length(tspan));
    
    for i = 1:length(tspan);
        if tspan(i) <= 2  
            targets(1,i) = -rEd(1,1);
            targets(2,i) = rEd(2,1);
            targetv(1,i) = 0;
            targetv(2,i) = 0;
        elseif 2 < tspan(i) && tspan(i) <= 2 + t_traj 
            targets(1,i) = -rEd(1,i-2000);
            targets(2,i) = rEd(2,i-2000);
            targetv(1,i) = (targets(1,i) - targets(1,i-1))/dt;
            targetv(2,i) = 0*(targets(2,i) - targets(2,i-1))/dt;

        elseif tspan(i) > 2 + t_traj
            targets(1,i) = -rEd(1,end);
            targets(2,i) = rEd(2,end);
            targetv(1,i) = 0;
            targetv(2,i) = 0;
        end
    end
    
    for i=1:num_step-1
        dz_tau=dynamics( tspan(i), z_out(:,i), p , targets, tspan, targetv);
        dz=dz_tau(1:4,:);
        tau_motors=dz_tau(5:6,:);
%         dz = dynamics(tspan(i), z_out(:,i), p , rEd);
        tau_out(:,i+1)=tau_motors;
        % Velocity update with dynamics
        new = dz*dt;
        z_out(3:4,i+1) = z_out(3:4,i) + new(3:4);
        z_out(1:2,i+1) = z_out(1:2,i);

        % Position update
        z_out(1:2,i+1) = z_out(1:2,i) + z_out(3:4,i+1)*dt;
    end

    %% Compute Energy
    if trial_figures
        E = energy_arm(z_out,p);
        figure(1); clf
        plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');
    end
    
    %% Compute hand position over time
    rE = zeros(2,length(tspan));
    vE = zeros(2,length(tspan));
    for i = 1:length(tspan)
        rE(:,i) = position_arm(z_out(:,i),p);
        vE(:,i) = velocity_arm(z_out(:,i),p);
    end
    if trial_figures
        figure(2); clf;
        plot(tspan,rE(1,:),'r','LineWidth',2)
        hold on
        plot(tspan,targets(1,:) ,'k--','LineWidth',3);
        plot(tspan,rE(2,:),'b','LineWidth',2)
        plot(tspan, targets(2,:) ,'k--','LineWidth',3);
        xlabel('Time (s)'); ylabel('Position (m)'); legend({'x','x_d','y','y_d'});
    
        figure(3); clf;
        plot(tspan,vE(1,:),'r','LineWidth',2)
        hold on
        plot(tspan,vE(2,:),'b','LineWidth',2)
        
        
        xlabel('Time (s)'); ylabel('Velocity (m)'); legend({'vel_x','vel_y'});

    
        figure(4)
        plot(tspan,z_out(1:2,:)*180/pi)
        legend('q1','q2');
        xlabel('Time (s)');
        ylabel('Angle (deg)');
        
        figure(5)
        plot(tspan,z_out(3:4,:)*180/pi)
        legend('q1dot','q2dot');
        xlabel('Time (s)');
        ylabel('Angular Velocity (deg/sec)');
    end
        
    %% Animate Solution
    if animate
        figure(6); clf;
        hold on
        plot(targets(1,:),targets(2,:),"k--");
        animateSol(tspan, z_out,p);
    end

    %% Calculate Momentum
    momentum = zeros(2,length(tspan));
    xmass_list = zeros(1,length(tspan));
    for i = 1:length(tspan)
        z = z_out(:,i);
        A = A_arm(z,p);
        J  = jacobian_arm(z,p); 
        M_op =inv( (J/A)*J' );
        xmass_list(i) = M_op(1,1);
        jointvels = [z(3) ; z(4)];
        %cartvels = J*jointvels;
        cartvels = vE(:,i);
%         velmag = sqrt(cartvels(1)^2 + cartvels(2)^2);
        momentum(:,i) = M_op*cartvels;
    end
    
    tot_momentum = (momentum(1,:).^2 + momentum(2,:).^2).^0.5;
    momentum_angle= atan2(momentum(2,:),momentum(1,:));
    if trial_figures
        figure(7); 
        plot(tspan,momentum(1,:),'r','LineWidth',2)
        title 'Momentum in X-direction'
        xlabel 'Time (s)'
        ylabel 'Momentum [kg*m/s]'
    
        figure(8); 
        plot(tspan,momentum(2,:),'b','LineWidth',2)
        title 'Momentum in Y-direction'
        xlabel 'Time (s)'
        ylabel 'Momentum [kg*m/s]'
    
        figure(9)
        plot(tspan,tot_momentum(1,:),'r','LineWidth',2)
        title 'Magnitude of Momentum'
        xlabel 'Time (s)'
        ylabel 'Momentum [kg*m/s]'
    
        figure(10)
        plot(tspan,tau_out,'LineWidth',2)
        xlabel('Time (s)')
        ylabel('Torque (N/m)')
        title('Torque over time')
        legend('Motor 1','Motor 2')
    
        figure(11)
        xvals = rE(1,:);
        xmom = momentum(1,:);
    
        index = find(xvals >= rEd(1,2));
        end_idx = index(1);
        index2 = find(tspan>=2.0);
        start_idx = index2(1);
        
        momentum_data = xmom(start_idx:end_idx);
        xdata = xvals(start_idx:end_idx);
        plot(xdata,momentum_data,'LineWidth',2)
        xlabel('X position [m]')
        ylabel('X Momentum [kg*m/s]')
        title('X momentum vs X position')
    end
    rex = rE(1,:);
    xplate = -.14;
    plateindices = find(rex<=xplate);
    
    %%region after hitting plate
    plate_region=round(0.2/dt);
    plate_region_indices=plateindices(1:plate_region);
    
    plateidx = plateindices(1);
    xmoment = momentum(1,:);
    peak_mom = abs(xmoment(plateidx));
    [peak_mom,I]=max(abs(xmoment(plate_region_indices)));
    
    peak_idx = find(momentum(1,:) == -peak_mom);

    %%calculates total momemntum magnitude (x and y) uncomment if you want
    %%to use this
%     peak_mom=sqrt(momentum(1,plate_region_indices(I))^2+momentum(2,plate_region_indices(I))^2);

    xmass = xmass_list(peak_idx);
    xvels = vE(1,:);
    xvel = -xvels(peak_idx);
    peakmass= max(xmass_list);
    peakxvel = -min(xvels);

    output=[peak_mom , peak_idx, xmass , xvel, peakmass, peakxvel];
end

function tau_limit=tau_constraint(tau,omega)
    %quadratic fit of torque speed curve
%     tau_omega_fit_quadratic=[0.0001,-0.0186,0.9274];
    %linear fit of torque and speed curve
    tau_omega_fit_linear=[-0.0132,0.8537];

    %choose which fit
    omega_fit=tau_omega_fit_linear;
%     omega_fit=tau_omega_fit_quadratic
    %find max torque value at the speed and grab it if the commanded torque is higher
    max_tau_fit=max(polyval(omega_fit,abs(omega)),zeros(size(tau)));
    tau_limit = min(max_tau_fit,abs(tau));
    tau_limit = max(tau_limit,zeros(size(tau_limit)));
    %assign negative torque values where needed
    tau_limit(tau<0)=-tau_limit(tau<0); 
end
function tau = control_law(t, z, p,targets, allt, targetv)
    % Controller gains
%     K_x = 100; % Spring stiffness X
%     K_y = 100; % Spring stiffness Y
%     D_x = 10;  % Damping X
%     D_y = 10;  % Damping Y

    K_x = 1000; % Spring stiffness X
    K_y = 500; % Spring stiffness Y
    D_x = 30;  % Damping X
    D_y = 100;  % Damping Y

    A = A_arm(z,p);
    J  = jacobian_arm(z,p); 
    G = Grav_arm(z,p);
    V = Corr_arm(z,p);
    Jdot = jacobian_dot_arm(z,p);

    M_op = inv(J/A*J');
    mu = M_op*J/A * V - M_op*Jdot* [z(3);z(4)];
    rho = M_op * J/A * G;

    K = [K_x , 0 ; 0 , K_y];
    D = [D_x , 0 ; 0 , D_y];

    idx = find(allt == t);
    rEd =  targets(:,idx);
    vEd = targetv(:,idx);

%     if t < 2.0
%         rEd = targets(:,1);
%         %vEd = [0 0];
%     else
%         rEd = targets(:,2); 
%         %vEd = [.5 0];
%     end
 
     %vEd = [0 0];
     aEd = [0 0];

    % Actual position and velocity 
    rE = position_arm(z,p);
    vE = velocity_arm(z,p);
    
    % Compute virtual foce for Question 1.4 and 1.5
    err_pos = rEd(1:2)- rE;
    err_pos = [err_pos(1) ; err_pos(2)];

%     epsilon = 0.05 ;
%     if abs(err_pos) < epsilon
%         vEd = [0 ; 0];
%     end

    err_vel = vEd(1:2) - vE;
    err_vel = [err_vel(1) ; err_vel(2)];
    aEd = [aEd(1) ; aEd(2)];
    f = M_op*(K*err_pos+D*err_vel)+ rho  ;
    tau = J' * f;
end


function dz_tau = dynamics(t,z,p,targets,tspan , targetv)
    % Get mass matrix
    A = A_arm(z,p);
    
    % Compute Controls
    tau_control = control_law(t,z,p,targets,tspan, targetv);
    tau=tau_constraint(tau_control,z(3:4));
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_arm(z,tau,p);
    
    % Solve for qdd.
    qdd = A\(b);
    dz = 0*z;
    
    % Form dz
    dz(1:2) = z(3:4);
    dz(3:4) = qdd;
    dz_tau=[dz;tau];
end

function animateSol(tspan, x,p)
    % Prepare plot handles
    hold on
    h_OB = plot([0],[0],'LineWidth',2);
    h_AC = plot([0],[0],'LineWidth',2);
    h_BD = plot([0],[0],'LineWidth',2);
    h_CE = plot([0],[0],'LineWidth',2);
    h_ellipse = plot(zeros(100),zeros(100),'LineWidth',2);
    
    xlabel('x'); ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-.3 .4 -.3 .1]);

    %Step through and update animation
    for i = 1:length(tspan)
        % skip frame.
        if mod(i,25)
            continue;
        end
        t = tspan(i);
        z = x(:,i); 
        keypoints = keypoints_arm(z,p);
        
        ellipse_pts=inertial_ellipse(z,p);

        rA = keypoints(:,1); 
        rB = keypoints(:,2);
        rC = keypoints(:,3);
        rD = keypoints(:,4);
        rE = keypoints(:,5);
        
        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        set(h_OB,'XData',[0 rB(1)]);
        set(h_OB,'YData',[0 rB(2)]);
        
        set(h_AC,'XData',[rA(1) rC(1)]);
        set(h_AC,'YData',[rA(2) rC(2)]);
        
        set(h_BD,'XData',[rB(1) rD(1)]);
        set(h_BD,'YData',[rB(2) rD(2)]);
        
        set(h_CE,'XData',[rC(1) rE(1)]);
        set(h_CE,'YData',[rC(2) rE(2)]);

        set(h_ellipse,'XData',ellipse_pts(1,:)+rE(1))
        set(h_ellipse,'YData',ellipse_pts(2,:)+rE(2))

        pause(.01)
    end
end