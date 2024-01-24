% Christian Casarotto - s223302 - DTU

% Takes as input a list of dimensione of the four robot links and wanted
% coordinates for the end effector, and wanted orientation of the end
% effector Phi

% Gives as output the four joint variables for the robot, q = [theta_1 ...]

% Note: works ONLY for robots with 4 elements, having 4 revolut joints 
%       whose free coordinates are four angles, where the first one is the
%       angle of rotation arounf Z, and the other three rotations around Y.
%       Here we are talking about global axes, where looking at the 3D
%       space Z points up, X to the right, and Y enters the plane.

function [theta_1, theta_2, theta_3, theta_4] = InverseKinAssignment(a_1,a_2,a_3,a_4,x_c,y_c,z_c,sinPhi)
        
    cosPhi = sqrt(1-sinPhi^2);
    theta_1 = atan2(y_c,x_c);
    r_prime = sqrt(x_c^2+y_c^2);
    r = r_prime-a_4*cosPhi;
    s_prime = z_c-a_1;
    s = s_prime-a_4*sinPhi;
    D = (r^2+s^2-a_2^2-a_3^2)/(2*a_2*a_3);
    theta_3 = atan2(-sqrt(1-D^2),D);
    theta_2 = atan2(s,r) - atan2(a_3*sin(theta_3),a_2+a_3*cos(theta_3));
    Phi = atan2(sinPhi,cosPhi);
    theta_4 = Phi - theta_2 - theta_3;

end



% if you were to use Phi

% function [theta_1, theta_2, theta_3, theta_4] = InverseKinAssignment(a_1,a_2,a_3,a_4,x_c,y_c,z_c,Phi)
% 
%     theta_1 = atan2(y_c,x_c);
%     r_prime = sqrt(x_c^2+y_c^2);
%     r = r_prime-a_4*cos(Phi);
%     s_prime = z_c-a_1;
%     s = s_prime-a_4*sin(Phi);
%     D = (r^2+s^2-a_2^2-a_3^2)/(2*a_2*a_3);
%     theta_3 = atan2(-sqrt(1-D^2),D);
%     theta_2 = atan2(s,r) - atan2(a_3*sin(theta_3),a_2+a_3*cos(theta_3));
%     theta_4 = Phi - theta_2 - theta_3;
% 
% end

