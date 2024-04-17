function Rx = quaternion_analytical(q_data_A, q_data_B, t_data_A, t_data_B)
    M = [];

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
    qx = V(:, 4);       % find unit quaternon
    Rx = quaternion_to_rotation(qx);


    %% find Px
    least_squares_test()

    

end