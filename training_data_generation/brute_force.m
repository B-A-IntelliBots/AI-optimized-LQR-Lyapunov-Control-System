%% ========================================================================
%  Brute Force Search for Optimal Control Gains (Kp, Kth)
%
%  Goal:
%  - Test a grid of possible proportional (Kp) and angular (Kth) gains.
%  - Compute the error metric for each gain pair using NNgo_to_start.m
%  - Refine the search resolution in two stages:
%       1. Coarse grid → to exclude poor gain regions.
%       2. Fine grid   → to precisely locate optimal gains.
%  - Select the gain pair that yields the smallest error.
%
%  Input:
%  - control_loop.m script/function (computes error metric and updates `e`).
%
%  Output:
%  - Kp  : Best proportional gain.
%  - Kth : Best angular gain.
%
%  Authors: [Ali Deeb, Bisher Alsaleh]
%  ========================================================================

clear e;    % Clear previous error matrix

%% ---------------- Coarse Search ----------------
flag = 1;                        % Flag used inside NNgo_to_start (mode = coarse)

% Define coarse search ranges
a = 0.001:0.1:1;                 % Range for Kp
b = 0.5:0.1:3;                   % Range for Kth
[KP, KTH] = meshgrid(a, b);

% Evaluate error metric for each (Kp, Kth) pair
for ii = 1:length(b)
    for jj = 1:length(a)
        Kp  = KP(ii,jj);
        Kth = KTH(ii,jj);
        control_loop;           % Must update error matrix "e"
    end
end

% Find worst-case error in coarse search (not used later, only for info/debug)
[iii, jjj] = find(e == max(e(:)));
max_error  = e(iii,jjj);

%% ---------------- Fine Search ----------------
clear e;                         % Reset error matrix
flag = 0;                        % Flag for NNgo_to_start (mode = fine)

% Define finer search ranges around same region
a = 0.001:0.05:1;                % Finer resolution for Kp
b = 0.5:0.05:3;                  % Finer resolution for Kth
[KP, KTH] = meshgrid(a, b);

% Evaluate error metric for each (Kp, Kth) pair
for ii = 1:length(b)
    for jj = 1:length(a)
        Kp  = KP(ii,jj);
        Kth = KTH(ii,jj);
        NNgo_to_start;           % Updates error matrix "e"
    end
end

% Find the optimal (minimum error) pair
[p1, p2] = find(e == min(e(:)));

% Assign the final optimal gains
Kp  = KP(p1,p2);
Kth = KTH(p1,p2);

