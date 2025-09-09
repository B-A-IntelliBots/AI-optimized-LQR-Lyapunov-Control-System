%% ========================================================================
%  Training Data Collection for Neural Network
%
%  Goal:
%  - Generate training data for a neural network controller.
%  - Target points are distributed within a unit circle around the robot.
%  - For each target point:
%       1. Compute error metrics for all possible control gains.
%       2. Select the gains that minimize the error (ground truth).
%       3. Save (x_target, y_target, Kp, Kth) as training data.
%
%  Output:
%  - Dataset saved in 'dataset_upper.csv'
%
%  Dependencies:
%  - brute_force.m (script/function that performs the gain search).
%
%  Authors: [Ali Deeb, Bisher Alsaleh]
%  ========================================================================

clear; clc;

%% ---------------- Simulation Parameters ----------------
dt       = 0.001;       % Simulation timestep [s]

%% ---------------- Reference Path Generation ----------------
% Generate target points distributed around a circle
ango     = pi/59;                           % Small angular offset
stepang  = [linspace(ango, pi-ango, 50), ...   % Upper half circle
            linspace(pi+ango, 2*pi-ango, 50)]; % Lower half circle
steplen  = linspace(0.1, 1, 10);            % Radii from 0.1 to 1.0 m

countt = 1;
for iq = 1:length(stepang)
    for jq = 1:length(steplen)
        % Target points in Cartesian coordinates
        x_reference(countt) = steplen(jq) * cos(stepang(iq));
        y_reference(countt) = steplen(jq) * sin(stepang(iq));
        countt = countt + 1;
    end
end

% Initial robot pose
x(1)     = 0;
y(1)     = 0;
theta(1) = 0;

%% ---------------- System Model ----------------
% NOTE: Variables "r" (wheel radius) and "d" (wheel base) 
% must be defined before running this section.

geometry = [1/r,  d/(2*r);     % Robot kinematics matrix
            1/r, -d/(2*r)];

% System dynamics matrices (from identification/modeling)
A = [-0.9099     0.2041   210.134   -47.1424;
      0.2041   -0.9099   -47.1424   210.134;
   -100.4333     0     -1537.52      0;
      0     -100.4333     0     -1537.52];

B = [  0       0;
        0       0;
      331.74    0;
        0    331.74 ];

C = [1 0 0 0;
     0 1 0 0];

D = [0 0;
     0 0];

% Augmented system (for state + error feedback)
A_ch            = zeros(6,6);
A_ch(1:4,1:4)   = A;
A_ch(5:end,1:4) = -C;

B_ch            = zeros(6,2);
B_ch(1:4,1:2)   = B;

% LQR design weights
Q = diag([0.1, 0.1, 1, 1, 1000, 1000]);   % State cost
R = diag([1000, 1000]);                   % Input cost

% LQR solution
[K, S, E] = lqr(A_ch, B_ch, Q, R);

% Closed-loop system matrices
A_new = A_ch - B_ch * K;
B_new = [0 0; 0 0; 0 0; 0 0; 1 0; 0 1];

%% ---------------- Dataset Generation ----------------
filename = 'dataset_upper.csv';
fileid   = fopen(filename, 'w');

% Loop through all target points
for ij = 1:(countt-1)
    % Call brute-force search for optimal control gains
    brute_force;
    
    % Save: [x_target, y_target, Kp, Kth]
    fprintf(fileid, '%f,%f,%f,%f\n', ...
        [x_reference(ij), y_reference(ij), Kp, Kth]);
end

fclose(fileid);

%% ---------------- Visualization ----------------
plot(xx, yy); 
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
title('Generated Target Points and Paths');
grid on;

