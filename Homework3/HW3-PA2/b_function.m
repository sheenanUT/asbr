% B_FUNCTION finds the b component of Ax=b which is used by the least squares function to find 
% the optimal translation vector x = Px
%     Input:
%         q_data_A: data set of robot hand rotations from base frame using quaternion representation
%     Output:
%         b_list: list of b component of Ax=b concatenated vertically

function b = b_function(Rx, t_data_A, t_data_B)
    b = [];
    
    for i = 1:size(t_data_A, 1)
        b = [b; Rx*(t_data_B(i, :))' - t_data_A(i, :)'];
    end
end