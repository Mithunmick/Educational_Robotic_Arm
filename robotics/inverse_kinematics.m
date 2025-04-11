function [theta] = inverse_kinematics(x, y, z, Phi)
    % Constants (link lengths) - modify according to your robot's parameters
    a2 = 93; % length of link 2
    a3 = 93; % length of link 3
    d1 = 50; % offset along Z axis
    a4 = 50; % length of link 4

    % Position of end effector in the base frame
    o_0_4 = [x; y; z];

    % Step 1: Calculate theta1 using x and y coordinates of o_0^4
    theta1_up = atan2(o_0_4(2), o_0_4(1)); 
    theta1 = theta1_up; % theta1 is the same for both configurations

    % Step 2: Determine the wrist center o_0^3 using the orientation angle Phi
    % Orientation components for the wrist based on end-effector angle Phi
    sinPhi = sin(Phi);
    cosPhi = cos(Phi);
    
    % Wrist center calculation: adjusting o_0_4 by link length a4 along Phi orientation
    o_0_3 = o_0_4 - a4 * [cos(theta1_up) * cosPhi; sin(theta1_up) * cosPhi; sinPhi];

    % Extract wrist center coordinates
    xc = o_0_3(1);
    yc = o_0_3(2);
    zc = o_0_3(3);

    % Step 3: Calculate r and s for the wrist center o_0^3
    r = sqrt(xc^2 + yc^2);
    s = zc - d1; 

    % Step 4: Calculate theta3 using the law of cosines for both configurations
    cos_theta3 = (r^2 + s^2 - a2^2 - a3^2) / (2 * a2 * a3);
    % Limit cos_theta3 within [-1, 1] to avoid errors due to numerical precision
    cos_theta3 = max(min(cos_theta3, 1), -1);
    
    % Elbow-up and elbow-down configurations
    % theta3 = atan2(-sqrt(1 - cos_theta3^2), cos_theta3); % Elbow-up
    theta3 = atan2(sqrt(1 - cos_theta3^2), cos_theta3); 

    % Step 5: Calculate theta2 for both configurations
    gamma = atan2(s, r); 
    beta = atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));

    theta2 = gamma - beta;

    % Step 6: Calculate theta4 based on the orientation constraint for both configurations
    theta4 = Phi - (theta2 + theta3); % Elbow-down orientation constraint

    theta = [theta1 theta2 theta3 theta4];
    
end