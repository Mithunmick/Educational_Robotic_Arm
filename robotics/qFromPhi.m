function q = qFromPhi(phi, P0c, R)
    % computeSingleInverseKinematics calculates inverse kinematics for a single point
    %
    % Inputs:
    %   phi - angle in radians to define a point on the circular path
    %   P0c - center point of the circle in 3D space [x; y; z]
    %   R   - radius of the circle
    %
    % Output:
    %   q   - vector of joint angles [theta1, theta2, theta3, theta4] for the point
    
    % Calculate the single point on the circular path in 3D
    P = P0c + R * [0; cos(phi); sin(phi)];
    
    % Call inverse kinematics function to compute joint angles for this point
    % Assuming inverse_kinematics function returns [theta1, theta2, theta3, theta4]
    q = IK(P(1), P(2), P(3), phi);
end
