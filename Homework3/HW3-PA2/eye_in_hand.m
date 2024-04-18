% EYE_IN_HAND calibrates transformation matrix of the camera to the robot hand
%     Inputs:
%         q_data_A: data set of robot hand rotations from base frame represented with quaternion
%         q_data_B: data set of robot camera rotations from fixed frame represented with quaternion
%         t_data_A: data set of robot hand translations from base frame
%         t_data_B: data set of robot camera rotations from fixed frame
%     Outputs:
%         X: 4x4 calibrated transformation matrix of the camera to the robot hand

function X = eye_in_hand(q_data_A, q_data_B, t_data_A, t_data_B)
    
    %% find Rx
    for i = 1:size(q_data_A, 1)
        % scalar component of quaternion
        scalarA = q_data_A(i, 1);
        scalarB = q_data_B(i, 1);
        % vector component of quaternion
        vectorA = q_data_A(i, 2:4)';
        vectorB = q_data_B(i, 2:4)';
        
        % generate quaternion matrix
        quaternionMatrix = [
            scalarA-scalarB, -(vectorA-vectorB)';
            (vectorA-vectorB), (scalarA-scalarB)*eye(3) + v2skew((vectorA+vectorB)');
        ];

        M = [M; quaternionMatrix];       % concatenate quaternion matrix vertically
    end

    % find Rx using unit quaternion
    [U, sigma, V] = svd(M);
    qx = (V(:, 4))';       % find unit quaternon
    Rx = quaternion_to_rotation(qx);


    %% find Px
    [px, flag] = lsqr(a_function(q_data_A), b_function(Rx, t_data_A, t_data_B));

    % generate the unknown transformation X
    X = [
        Rx, px;
        zeros(1, 3), 1;
    ];
end