function [zyz, zyz_Alt] = r2zyz(R)
%R2ZYZ Converts rotation matrix to ZYZ Euler angle representation
%   Inputs:
%       R = 3x3 rotation matrix, SO(3)
%   Outputs:
%       zyz = Vector of Euler angles for 0 < theta < pi
%       zyzAlt = Vector of Euler angles for -pi < theta < 0
%   Outputs are structured as [phi, theta, psi] where:
%       phi = Angle of first Z-rotation, radians
%       theta = Angle of Y-rotation, radians
%       psi = Angle of second Z-rotation, radians

    % Validate inputs
    if ~is_rotation_matrix(R)
        error("Input R is not a valid SO(3) rotation matrix");
    end

    % Singularity case: R = I
    if ismembertol(R, eye(3), 1e-4)
        phi = 0; theta = 0; psi = 0;
        phi_A = 0; theta_A = 0; psi_A = 0;
    % Pure z-rotation
    elseif R(1,3) == 0 && R(2,3) == 0
        % Can only determine sum of phi and psi (alpha)
        % For primary output, assume phi = 0, psi = alpha
        % For alt output, let phi = 2*alpha, psi = -alpha
        % Follows convention of base MatLab function
        alpha = asin(R(2,1));
        phi = 0; theta = 0; psi = alpha;
        phi_A = 2 * alpha; theta_A = 0; psi_A = -alpha;
    else
        % theta in (0, pi)
        phi = atan2(R(2,3), R(1,3));
        theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
        psi = atan2(R(3,2), -R(3,1));
    
        % theta in (-pi, 0)
        phi_A = atan2(-R(2,3), -R(1,3));
        theta_A = atan2(-sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
        psi_A = atan2(-R(3,2), R(3,1));
    end

    zyz = [phi theta psi];
    zyz_Alt = [phi_A theta_A psi_A];
    
end