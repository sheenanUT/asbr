% ROTATIONMATRIX converts the quaternion representation of a rotation to a rotation matrix
%     Input:
%         quaternion: 1x4 quaternion representation of a rotation
%     Output:
%         rotationMatrix: 3x3 rotation matrix

function rotationMatrix = quaternion_to_rotation(quaternion)
    % split quaternion into elements
    q0 = quaternion(1);
    q1 = quaternion(2);
    q2 = quaternion(3);
    q3 = quaternion(4);
    
    % find rotation matrix
    rotationMatrix = [
        q0^2 + q1^2 - q2^2 - q3^2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
        2*(q1*q2 + q0*q3), q0^2 - q1^2 + q2^2 - q3^2, 2*(q2*q3 - q0*q1);
        2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0^2 - q1^2 - q2^2 + q3^2;
    ];
end