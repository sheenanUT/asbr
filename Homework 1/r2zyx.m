function [zyx, zyx_Alt] = r2zyx(R)
%R2ZYX Converts rotation matrix to ZYX (Roll-Pitch-Yaw) Euler angle
%representation
%   Inputs:
%       R = 3x3 rotation matrix, SO(3)
%   Outputs:
%       zyx = Vector of Euler angles for -pi/2 < theta < pi/2
%       zyxAlt = Vector of Euler angles for pi/2 < theta < 3pi/2
%   Outputs are structured as [roll, pitch, yaw] where:
%       roll = Angle of Z-rotation, radians
%       pitch = Angle of Y-rotation, radians
%       yaw = Angle of X-rotation, radians

    % Validate inputs
    if ~is_rotation_matrix(R)
        error("Input R is not a valid SO(3) rotation matrix");
    end

    % theta in (-pi/2, pi/2)
    roll = atan2(R(2,1), R(1,1));
    pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    yaw = atan2(R(3,2), R(3,3));

    % theta in (pi/2, 3pi/2)
    roll_A = atan2(-R(2,1), -R(1,1));
    pitch_A = atan2(-R(3,1), -sqrt(R(3,2)^2 + R(3,3)^2));
    yaw_A = atan2(-R(3,2), -R(3,3));

    zyx = [roll pitch yaw];
    zyx_Alt = [roll_A pitch_A yaw_A];

end