function [quat] = r2quat(R)
%R2QUAT Converts rotation matrix to quaternion
%   Inputs:
%       R = 3x3 rotation matrix, SO(3)
%   Outputs:
%       quat = 1x4 vector representing quaternion of R
%   quat is structured as [q0 q1 q2 q3] where
%   Q = q0 + q1*i + q2*j + q3*k

    % Validate inputs
    if ~is_rotation_matrix(R)
        error("Input R is not a valid SO(3) rotation matrix");
    end

    % Get axis-angle representation of R
    axang = r2axisangle(R);
    w = axang(1:3);
    theta = axang(4);

    % Convert to quaternion components
    q0 = cos(theta / 2);
    q_vector = w * sin(theta / 2);
    q1 = q_vector(1); q2 = q_vector(2); q3 = q_vector(3);
    
    % Return quaternion
    quat = [q0, q1, q2, q3];
end