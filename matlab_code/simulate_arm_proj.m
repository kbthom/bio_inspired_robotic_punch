function simulate_arm()
    addpath('auto_derived\')
    addpath('animate\')
    addpath('modeling\')
    %% Define fixed paramters
    m1 =.0393 + .2;         m2 =.0368; 
    m3 = .00783;            m4 = .0155;
    I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
    I3 = 9.25 * 10^-6;      I4 = 22.176 * 10^-6;
    l_OA=.011;              l_OB=.042; 
    l_AC=.096;              l_DE=.091;
    l_O_m1=0.032;           l_B_m2=0.0344; 
    l_A_m3=0.0622;          l_C_m4=0.0610;
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81;    
   
    
%     restitution_coeff = 0.;
%     friction_coeff = 10;
%     ground_height = -0.13;
    %% Parameter vector
    p   = [m1 m2 m3 m4 I1 I2 I3 I4 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_OA l_OB l_AC l_DE g]';
       
    %% Simulation Parameters Set 2 -- Operational Space Control
    p_traj.omega = 3;
    p_traj.x_0   = 0;
    p_traj.y_0   = -.125;
    p_traj.r     = 0.025;
    
    %% Perform Dynamic simulation
    dt = 0.001;
    tf = 5;
    num_step = floor(tf/dt);
    tspan = linspace(0, tf, num_step); 
    z0 = [-pi/4; pi/2; 0; 0];
    z_out = zeros(4,num_step);
    z_out(:,1) = z0;

    rEd = [0 , 0; -.1 , -(.042 + .096 + .091)];
    targets = zeros(2,length(tspan));
    for i = 1:length(tspan);
        if tspan(i) < 2  
            targets(1,i) = rEd(1,1);
            targets(2,i) = rEd(2,1);
        else
            targets(1,i) = rEd(1,2);
            targets(2,i) = rEd(2,2);
        end
    end
    
    for i=1:num_step-1
        dz = dynamics(tspan(i), z_out(:,i), p, p_traj,rEd);

        % Velocity update with dynamics
        new = dz*dt;
        z_out(3:4,i+1) = z_out(3:4,i) + new(3:4);
        z_out(1:2,i+1) = z_out(1:2,i);

%         % Velocity update with impact
%         qdot = discrete_impact_contact(z_out(:,i+1), p, restitution_coeff, friction_coeff, ground_height);
%         z_out(3:4,i+1) = qdot;

        % Position update
        z_out(1:2,i+1) = z_out(1:2,i) + z_out(3:4,i+1)*dt;
    end


    
    %% Compute Energy
    E = energy_arm(z_out,p);
    figure(1); clf
    plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');
    
    %% Compute foot position over time
    rE = zeros(2,length(tspan));
    vE = zeros(2,length(tspan));
    for i = 1:length(tspan)
        rE(:,i) = position_arm(z_out(:,i),p);
        vE(:,i) = velocity_arm(z_out(:,i),p);
    end
    
    figure(2); clf;
    plot(tspan,rE(1,:),'r','LineWidth',2)
    hold on
    %plot(tspan,p_traj.x_0 + p_traj.r * cos(p_traj.omega*tspan) ,'r--');
    plot(tspan,targets(1,:) ,'k--','LineWidth',3);
    plot(tspan,rE(2,:),'b','LineWidth',2)
    %plot(tspan,p_traj.y_0 + p_traj.r * sin(p_traj.omega*tspan) ,'b--');
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
    
    %% Animate Solution
    figure(6); clf;
    hold on
   
  
%     % Target traj
%     TH = 0:.1:2*pi;
%     plot( p_traj.x_0 + p_traj.r * cos(TH), ...
%           p_traj.y_0 + p_traj.r * sin(TH),'k--'); 
     plot(rEd(1,1),rEd(2,1),'o');
     plot(rEd(1,2),rEd(2,2),'o');
%     % Ground Q2.3
%     plot([-.2 .2],[ground_height ground_height],'k'); 
    
    
    
    animateSol(tspan, z_out,p);

    
    momentum = zeros(1,length(tspan));
    for i = 1:length(tspan)
        z = z_out(:,i);
        A = A_arm(z,p);
        J  = jacobian_arm(z,p); 
        M_op =inv( (J/A)*J' );
        jointvels = [z(3) ; z(4)];
        cartvels = J*jointvels;
        velmag = sqrt(cartvels(1)^2 + cartvels(2)^2);
        momentum(1,i) = abs(M_op(2,2) * cartvels(2));
    end

    figure(7); 
    plot(tspan,momentum,'r','LineWidth',2)
    title 'Momentum'
    xlabel 'Time (s)'
    ylabel 'Momentum [kg*m/s]'

end

function tau = control_law(t, z, p, p_traj,targets,tau_constraint)
    % Controller gains, Update as necessary for Problem 1
    K_x = 100; % Spring stiffness X
    K_y = 100; % Spring stiffness Y
    D_x = 10;  % Damping X
    D_y = 10;  % Damping Y

    A = A_arm(z,p);
    J  = jacobian_arm(z,p); 
    G = Grav_arm(z,p);
    V = Corr_arm(z,p);
    Jdot = jacobian_dot_arm(z,p);

    M_op = inv(J/A*J');
    mu = M_op * J * inv(A)* V - M_op * Jdot* [z(3) ; z(4)];
    rho = M_op * J * inv(A) * G;

    K = [K_x , 0 ; 0 , K_y];
    D = [D_x , 0 ; 0 , D_y];

    if t < 2.0
        rEd = targets(:,1);
    else
        rEd = targets(:,2);
    end
 
     vEd = [0 0];
     aEd = [0 0];

    % Actual position and velocity 
    rE = position_arm(z,p);
    vE = velocity_arm(z,p);
    
    % Compute virtual foce for Question 1.4 and 1.5
     err_pos = rEd(1:2)- rE;
     err_pos = [err_pos(1) ; err_pos(2)];
     err_vel = vEd(1:2) - vE;
     err_vel = [err_vel(1) ; err_vel(2)];
     aEd = [aEd(1) ; aEd(2)];
 
%     f  = M_op * (aEd + K*err_pos + D*err_vel) + mu + rho;
     f = M_op*(K*err_pos+D*err_vel);
   
    tau = J' * f;
    tau = min(tau,tau_constraint);
end


function dz = dynamics(t,z,p,p_traj,targets)
    % Get mass matrix
    A = A_arm(z,p);
    
    % Compute Controls
    tau_constraint = [0.83385;0.83385];
    tau = control_law(t,z,p,p_traj,targets,tau_constraint);
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_arm(z,tau,p);
    
    % Solve for qdd.
    qdd = A\(b);
    dz = 0*z;
    
    % Form dz
    dz(1:2) = z(3:4);
    dz(3:4) = qdd;
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
    axis([-.2 .2 -.3 .1]);

    %Step through and update animation
    for i = 1:length(tspan)
        % skip frame.
        if mod(i,10)
            continue;
        end
        t = tspan(i);
        z = x(:,i); 
        keypoints = keypoints_arm(z,p);
        
        ellipse_pts=inertial_ellipse(z,p);

        rA = keypoints(:,1); % Vector to base of cart
        rB = keypoints(:,2);
        rC = keypoints(:,3); % Vector to tip of pendulum
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