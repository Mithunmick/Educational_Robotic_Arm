function [theta] = IK(x_c,y_c,z_c,Phi)

    a_2 = 93; % length of link 2
    a_3 = 93; % length of link 3
    a_1 = 50; % offset along Z axis
    a_4 = 50; % length of link 4

    theta_1 = atan2(y_c,x_c);
    r_prime = sqrt(x_c^2+y_c^2);
    r = r_prime-a_4*cos(Phi);
    s_prime = z_c-a_1;
    s = s_prime-a_4*sin(Phi);
    D = (r^2+s^2-a_2^2-a_3^2)/(2*a_2*a_3);
    D = max(min(D, 1), -1);
    theta_3 = atan2(-sqrt(1-D^2),D);
    theta_2 = atan2(s,r) - atan2(a_3*sin(theta_3),a_2+a_3*cos(theta_3));
    theta_4 = Phi - theta_2 - theta_3;
    theta = [theta_1, theta_2, theta_3, theta_4];

end