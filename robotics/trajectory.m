function C = calculateTrajectoryCoefficients(v, q, J)
    % calculateTrajectoryCoefficients computes the coefficient vector C for polynomial
    % trajectory generation based on joint positions, velocities, and the Jacobian.
    %
    % Inputs:
    %   v - 6x1 vector of end-effector velocities
    %   q - Nx4 matrix of joint angle configurations (with at least 37 rows)
    %   J - Jacobian matrix evaluated at the end effector
    %
    % Output:
    %   C - 6x1 coefficient vector for trajectory generation
    
    % Extract specific joint configurations for time intervals
    q0 = q(1, :);
    q9 = q(9, :);
    q18 = q(18, :);
    q27 = q(27, :);
    q36 = q(36, :);
    
    % Define time matrix for the trajectory (start and end time points)
    time = @(tin, ta) [1 tin tin^2 tin^3 tin^4 tin^5;
                       0 1 2*tin 3*tin^2 4*tin^3 5*tin^4;
                       0 0 2 6*tin 12*tin^2 20*tin^3;
                       1 ta ta^2 ta^3 ta^4 ta^5;
                       0 1 2*ta 3*ta^2 4*ta^3 5*ta^4;
                       0 0 2 6*ta 12*ta^2 20*ta^3];
                   
    % Generate time matrix for interval [0, 2] seconds
    time_matrix = time(0, 2);
    
    % Compute joint velocity vector q_dot using the Jacobian's pseudoinverse
    q_dot1 = pinv(J) * v;  % Joint velocities at q0
    
    % Define Q vector with boundary conditions
    Q = @(qin, vin, ain, qa, va, aa) [qin; vin; ain; qa; va; aa];
    Q1 = Q(q0(end), 0, 0, q9(end), q_dot1(end), 0);  % For one of the joints' trajectory
    
    % Calculate coefficient vector C for the trajectory polynomial
    C = inv(time_matrix) * Q1;
end
