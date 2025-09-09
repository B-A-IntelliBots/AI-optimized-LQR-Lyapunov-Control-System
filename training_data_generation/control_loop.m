%% ========================================================================
%  Control Loop Simulation with Candidate Gains (Kp, Kth)
%
%  Goal:
%  - Simulate the robot starting at (0,0) and moving toward a target point
%    (x_start, y_start) using control gains (Kp, Kth).
%  - At each timestep:
%       1. Compute control inputs (linear and angular velocities).
%       2. Map to wheel reference velocities.
%       3. Update robot dynamics (linear model + kinematics).
%  - After the run, compute an error metric (path deviation & timing).
%  - Save error metric in matrix e(ii,jj) for the current (Kp, Kth).
%
%  Dependencies:
%  - Called inside brute_force.m
%  - Uses system matrices (A_new, B_new), geometry, dt, time, etc.
%
%  Output:
%  - Updates global matrix e(ii,jj) with error value.
%
%  Authors: [Ali Deeb, Bisher Alsaleh]
%  ========================================================================

%% ---------------- Target Initialization ----------------
x_start = x_reference(ij);   % Target X position
y_start = y_reference(ij);   % Target Y position

% Reset trajectory arrays
clear xx yy th
xx(1) = 0;    % Initial X position
yy(1) = 0;    % Initial Y position
th(1) = 0;    % Initial orientation

% Initial error to target
ex_f  = x_start - xx(1);
ey_f  = y_start - yy(1);
d_f   = sqrt(ex_f^2 + ey_f^2);              % Initial distance
eth_f = atan2(ey_f, ex_f) - th(1);          % Initial heading error

% Normalize heading error to [-pi, pi]
if (eth_f > pi),   eth_f = eth_f - 2*pi; end
if (eth_f < -pi),  eth_f = eth_f + 2*pi; end

% State vector initialization
z = zeros(6,1);

%% ---------------- Control Loop ----------------
for j = 1:length(time)-1
    
    % Stop condition: target reached (small distance threshold)
    if (d_f < 0.0099)
        break;
    end
    
    % --- Control law (linear + angular velocities) ---
    v_f = Kp * d_f * cos(eth_f);                  % Forward velocity
    w_f = Kp * (cos(eth_f)*sin(eth_f)) + Kth*eth_f; % Angular velocity
    
    % --- Convert to wheel reference velocities ---
    temp1   = geometry * [v_f; w_f];
    wr_fref = temp1(1);
    wl_fref = temp1(2);
    
    % --- System dynamics update ---
    dz = A_new * z + B_new * [wr_fref; wl_fref];
    z  = z + dz * dt;
    
    % --- Kinematic update (robot pose) ---
    Kinematic_f = r * [cos(th(j))/2 cos(th(j))/2;
                       sin(th(j))/2 sin(th(j))/2;
                       1/d -1/d];
    temp2 = Kinematic_f * z([1 2],1);
    dxx   = temp2(1);
    dyy   = temp2(2);
    dth   = temp2(3);
    
    % Update position and orientation
    temp3     = [xx(j); yy(j); th(j)] + dt * [dxx; dyy; dth];
    xx(j+1)   = temp3(1);
    yy(j+1)   = temp3(2);
    th(j+1)   = temp3(3);
    
    % Normalize orientation to [-pi, pi]
    if (th(j+1) > pi),   th(j+1) = th(j+1) - 2*pi; end
    if (th(j+1) < -pi),  th(j+1) = th(j+1) + 2*pi; end
    
    % --- Update errors ---
    ex_f  = x_start - xx(j+1);
    ey_f  = y_start - yy(j+1);
    d_f   = sqrt(ex_f^2 + ey_f^2);
    eth_f = atan2(ey_f, ex_f) - th(j+1);
end

%% ---------------- Error Metric Calculation ----------------
% Path deviation calculation (geometric error)
L2 = (x_start - xx).^2 + (y_start - yy).^2;
L1 = (xx(1) - xx).^2 + (yy(1) - yy).^2;
LL = sqrt((x_start - xx(1))^2 + (y_start - yy(1))^2);

k1 = (LL.^2 - L2 + L1) ./ (2*LL);
k2 = sqrt(L1 - k1.^2);

% Two modes: flag = 1 → simple error, flag = 0 → detailed error
if (flag)
    % Simple error metric = average path deviation
    e(ii,jj) = real(sum(k2)) / length(k2);
else
    % Advanced error metric
    dd     = sqrt(x_start^2 + y_start^2);   % Distance to target
    timee  = dt * length(k2);               % Time taken
    er2    = abs(timee - 3.5*dd);           % Time deviation
    
    % Weighted sum of error terms:
    %  1. Path deviation
    %  2. Final heading error
    %  3. Timing penalty
    e(ii,jj) = 1000 * ( ...
        1.5 * (real(sum(k2))/length(k2)) / max_error + ...
        2 * min(abs(th(end)-atan2(y_start,x_start)), ...
                2*pi - abs(th(end)-atan2(y_start,x_start))) / pi + ...
        2 * tansig(er2 * 0.1) );
end

