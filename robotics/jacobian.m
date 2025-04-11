function [Jee, Jcam] = jacobian(q)
    % Define the DH transformation function
    T_ = @(th, d, a, al) [cos(th), -sin(th)*cos(al), sin(th)*sin(al), a*cos(th);
                          sin(th), cos(th)*cos(al), -cos(th)*sin(al), a*sin(th);
                          0, sin(al), cos(al), d;
                          0, 0, 0, 1];
    
    % Define DH parameters based on the given table
    T01 = T_(q(1), 50, 0, pi/2);
    T12 = T_(q(2), 0, 93, 0);
    T23 = T_(q(3), 0, 93, 0);
    T34 = T_(q(4), 0, 50, 0);

    % Compute cumulative transformations to frame 4
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;  % Transformation to frame 4 (end effector)
    
    % Additional transformation from frame 4 to frame 5 (camera frame)
    T45 = [1 0 0 -15; 
           0 1 0 45; 
           0 0 1 0; 
           0 0 0 1];
    T05 = T04 * T45; % Transformation to camera frame (frame 5)
    
    % Axes z and Origins o for Jacobian computation
    z0 = [0; 0; 1];
    z1 = T01(1:3, 3);
    z2 = T02(1:3, 3);
    z3 = T03(1:3, 3);

    o0 = [0; 0; 0];
    o1 = T01(1:3, 4);
    o2 = T02(1:3, 4);
    o3 = T03(1:3, 4);
    o4 = T04(1:3, 4); % Position of the end effector
    o5 = T05(1:3, 4); % Position of the camera frame

    % Jacobian columns for the end effector
    J1_ee = [cross(z0, (o4 - o0)); z0];
    J2_ee = [cross(z1, (o4 - o1)); z1];
    J3_ee = [cross(z2, (o4 - o2)); z2];
    J4_ee = [cross(z3, (o4 - o3)); z3];

    % Construct Jacobian matrix for the end effector
    Jee = [J1_ee, J2_ee, J3_ee, J4_ee];
    
    % Jacobian columns for the camera frame
    J1_cam = [cross(z0, (o5 - o0)); z0];
    J2_cam = [cross(z1, (o5 - o1)); z1];
    J3_cam = [cross(z2, (o5 - o2)); z2];
    J4_cam = [cross(z3, (o5 - o3)); z3];

    % Construct Jacobian matrix for the camera
    Jcam = [J1_cam, J2_cam, J3_cam, J4_cam];
end
