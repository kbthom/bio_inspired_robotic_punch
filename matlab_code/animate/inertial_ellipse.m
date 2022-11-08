function ellipse_pts=inertial_ellipse(z,p)
    
    A = A_arm(z,p);
    J  = jacobian_arm(z,p);
    M_op=inv(J/A*J');
    % get eigenvectors and corresponding values
    [V,D] = eig(M_op); % columns of V are eigenvectors, diagonal elements of D are eigenvalues
    
    % using parametric equations of ellipse, calculate ellipse points relative to foot position
    th_ellipse = -atan2(V(1,1),V(2,1)); % angle between first eigenvector and positive x axis
    l_x = 0.01*(1/D(1,1)); % TODO: better way to implement scaling of the ellipse?
    l_y = 0.01*(1/D(2,2)); 
    ii = linspace(0, 2*pi, 100);
    xpts = (l_x*cos(ii))*cos(th_ellipse) - (l_y*sin(ii))*sin(th_ellipse);
    ypts = (l_x*cos(ii))*sin(th_ellipse) + (l_y*sin(ii))*cos(th_ellipse);
    
    ellipse_pts = [xpts;ypts];
