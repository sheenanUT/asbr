[q_data_A, q_data_B, t_data_A, t_data_B] = data_quaternion();
X = eye_in_hand(q_data_A, q_data_B, t_data_A, t_data_B);
disp('Noise Free Data Set');
disp(X);