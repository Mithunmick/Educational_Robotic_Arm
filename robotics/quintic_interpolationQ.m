function C_all = quintic_interpolationQ(tin, tA, qin,qdotin,qdotdotin, qA, qdotA,qdotdotA)
    % Initialize the time matrix function
    time = @(tin, ta) [1 tin tin^2 tin^3 tin^4 tin^5;
                       0 1 2*tin 3*tin^2 4*tin^3 5*tin^4;
                       0 0 2 6*tin 12*tin^2 20*tin^3;
                       1 ta ta^2 ta^3 ta^4 ta^5;
                       0 1 2*ta 3*ta^2 4*ta^3 5*ta^4;
                       0 0 2 6*ta 12*ta^2 20*ta^3];
    
    % Set up the time matrix
    A = time(tin, tA);
    
    % Number of joints (assumed from size of qin)
    num_joints = length(qin);
    
    % Initialize a matrix to store coefficients for each joint
    C_all = zeros(num_joints, 6); % Assuming 6 coefficients per joint

    
    % Loop over each joint to calculate coefficients
    for i = 1:num_joints
        % Calculate boundary velocities for current joint
        
        % Define the boundary condition vector for the current joint
        b = [qin(i); qdotin(i); qdotdotin(i); qA(i); qdotA(i); qdotdotA(i)];
        
        % Calculate coefficients for the current joint and store
        C_all(i, :) = (A\b)';
    end
end
