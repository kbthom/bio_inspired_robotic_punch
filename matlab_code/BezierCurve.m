function traj = BezierCurve(pts, ts)

    N = size(pts,2)-1;
    M = size(pts,1);
    if nargin < 3
        deriv_order = 0;
    end
    for i = 1:deriv_order
        pts = N * diff(pts')';
        N   = N-1;
    end
    vals = zeros(M,length(ts));
    for k = 0:N
        vals = vals + pts(:,k+1) * nchoosek(N,k) * ( ts.^k .* (1-ts).^(N-k));
    end
    traj = vals;
end

% ncont = length(ctrl_pt);
% traj = 0;
%     for i = 1:ncont
%         traj = traj  + (nchoosek(ncont,i))*tbez.^i*(1-tbez).^(ncont-i)*ctrl_pt(i); % compute return value. Write your code instead of 1.
%     end
