function [ps_goal, ts] = goal_circle(T, r, w, theta_stop)
%GOAL_CIRCLE Simulates a target position (p_goal) moving on a circular path
%   Inputs:
%       T = 4x4 transformation matrix for center of circle. Default
%       orientation is along the YZ plane with the norm vector along the 
%       x-axis, p_goal along z-axis at t=0
%       r = scalar, radius of circle in meters
%       w = scalar, angular velocity of p_goal, rads per second
%       theta_stop = scalar, final rotation angle, radians
%   Outputs:
%       ps_goal = nx3 matrix where each row i is the position vector for
%       p_goal at time t(i)
%       ts = nx1 vector of times correspoding with ps_goal. The time step
%       is chosen such that p_goal travels no more than 1 cm per step, with
%       a maximum step of 1 second

% Determine the time step size
% radius step = r*w*dt <= 0.01
% dt <= 0.01 / (rw)
% Max 1 second
dt = min(1, 0.01 / (r * w));

dtheta = dt * w;    % Angular step size
thetas = 0 : dtheta : theta_stop;

% Initial position is [0 0 r] in circle-frame
ts = zeros([length(thetas), 1]);
ps_goal = zeros([length(thetas), 3]);
p_init = [0; 0; r];

for i = 1 : length(thetas)
    if i > 1
        ts(i) = ts(i - 1) + dt;
    end
    theta = thetas(i);
    % First calculate positions in circle-frame, then transform to space-frame
    % Rotate p_init about x by theta in circle-frame
    rotation = axisangle2r([1, 0, 0, theta]);
    p_circle = rotation * p_init;
    % Transform circle frame to space frame
    p_goal_h = T * [p_circle; 1];
    ps_goal(i, :) = p_goal_h(1:3)';
end

end