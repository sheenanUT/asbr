% A_FUNCTION finds the A components of Ax=b which is used by the least squares function to find 
% the optimal translation vector x = Px
%     Input:
%         q_data_A: data set of robot hand rotations from base frame using quaternion representation
%     Output:
%         a: list of A components of Ax=b concatenated vertically

function a = a_function(q_data_A)
    a = [];

    for i = 1:size(q_data_A, 1)
        Ra_i = quaternion_to_rotation(q_data_A(i, :));
        a = [a; (Ra_i-eye(3))];     % concatenate
    end
end
