% Project Assignment: Educational Robotic Arm

clc; clear all
syms x y z

% Robot dimensions [mm]
a_1 = 50;  a_2 = 93;  a_3 = 93;  a_4 = 50;



%% Problem 4: Find the Jacobian

% Jacobian will be of the form [z_i-1 x (o_n - o_i-1) as all revolut joints
%                               z_i-1               ] 

% Angles we want to study
angles = [0 pi/2 pi 3*pi/2 2*pi];

% First compute the two Jacobians as symbolic
syms theta_1 theta_2 theta_3 theta_4

% Updating DH parameters - Now we need to use the pi/2 ???
DH_parameters = [theta_1    a_1   0      pi/2
                 theta_2    0     a_2    0
                 theta_3    0     a_3    0
                 theta_4    0     a_4    0];

% Extracting single rotation matrices for each set of axes
T_01 = DH(DH_parameters(1,:));
T_02 = DH(DH_parameters(1:2,:));
T_03 = DH(DH_parameters(1:3,:));
T_04 = DH(DH_parameters(1:4,:));

% Extract values of axis origin and z axis orientation from rotation matrix
o_0 = [0 0 0].';       z_0 = [0 0 1].';
o_1 = T_01(1:3,4);     z_1 = T_01(1:3,3);
o_2 = T_02(1:3,4);     z_2 = T_02(1:3,3);
o_3 = T_03(1:3,4);     z_3 = T_03(1:3,3);
o_4 = T_04(1:3,4);     z_4 = T_04(1:3,3);

% Creating J matrice
J_EndEffector = [cross(z_0,(o_4-o_0))  cross(z_1,(o_4-o_1))  cross(z_2,(o_4-o_2))  cross(z_3,(o_4-o_3))
                         z_0                   z_1                   z_2                   z_3         ];

% The Jacobian for the robot camera is built in the same way, the axes
% are also the same, but the final position is now o_5, for which we
% need to find the coordinates

% Redefine rotation matrices
T_45 = [1 0 0 -15
        0 1 0  45
        0 0 1  0
        0 0 0  1];
T_05 = T_04*T_45;

% Compute missing axes origin
o_5 = T_05(1:3,4);     z_5 = z_4;

% Creating J matrice 
J_RobotCamera = [cross(z_0,(o_5-o_0))  cross(z_1,(o_5-o_1))  cross(z_2,(o_5-o_2))  cross(z_3,(o_5-o_3)) 
                         z_0                   z_1                   z_2                   z_3         ];


% So we now have the two Jacobians J, one for the end effector and one for
% the camera. J_EndEffector and J_RobotCamera.

% Compute a loop for each of the angles to find the needed joint variables
% q = [theta_1 theta_2 theta_3 theta_4]
% Restate circle coordinates [mm]
p_0_c = [150 0 120];  
R = 32;   
x_04_3 = 0; % of x from 0 to 4, third element (sin Phi) - Horizontal

% Initialize array to collect joint angles
q = zeros(4,length(angles));
for i=1:length(angles)

    % Updating the robot configuration
    phi = angles(i);
    coord = p_0_c + R*[0 cos(phi) sin(phi)];

    % Coordinates of wrist center and its rotation (phi) - all global
    x_c = coord(1); y_c = coord(2); z_c = coord(3);

    % Compute inverse kinematics with external function
    [theta_1, theta_2, theta_3, theta_4] = InverseKinAssignment(a_1,a_2,a_3,a_4,x_c,y_c,z_c,x_04_3);
    
    % Collect joint variables, used for tha Jacobians and for Problem 6
    q_current = [theta_1 theta_2 theta_3 theta_4].';
    q(:,i) = q_current;

end

% Declare as simbolic, for substitution
syms theta_1 theta_2 theta_3 theta_4

% Extracting the matrices for end effector
J_EndEffector_angle1 = double(subs(J_EndEffector, [theta_1 theta_2 theta_3 theta_4].', q(:,1)));
J_EndEffector_angle2 = double(subs(J_EndEffector, [theta_1 theta_2 theta_3 theta_4].', q(:,2)));
J_EndEffector_angle3 = double(subs(J_EndEffector, [theta_1 theta_2 theta_3 theta_4].', q(:,3)));
J_EndEffector_angle4 = double(subs(J_EndEffector, [theta_1 theta_2 theta_3 theta_4].', q(:,4)));

% Extracting the matrices for robot camera
J_RobotCamera_angle1 = double(subs(J_RobotCamera, [theta_1 theta_2 theta_3 theta_4].', q(:,1))); 
J_RobotCamera_angle2 = double(subs(J_RobotCamera, [theta_1 theta_2 theta_3 theta_4].', q(:,2))); 
J_RobotCamera_angle3 = double(subs(J_RobotCamera, [theta_1 theta_2 theta_3 theta_4].', q(:,3))); 
J_RobotCamera_angle4 = double(subs(J_RobotCamera, [theta_1 theta_2 theta_3 theta_4].', q(:,4))); 



%% Problem 5: Joint velocities (inverse velocity)

% COMMENTED THE OLD VERSION

% % Velocites of the end effector to be imposed
% v_0_4 = [0,-3,0].'; % [mm/s] 
% 
% % We only have the linear velocities, as the angular velocities are also
% % given by the rotation of the joints... therefore we are not interested 
% % in those, but only on the linear velocites
% xi = v_0_4;
% 
% % Isolating the Jacobian we are interested in
% J = J_EndEffector_angle2;
% % and only the part we are interested in. We only are interested in linear
% % velocities, therefore we will only use the first half of the Jacobian
% J = double(J(1:3,:));
% 
% % Performing the required rank test
% if rank(J)~=rank([J xi]) 
%     disp('Rank test non valid, J cannot be inverted')
% end
% 
% % Calculate the pseudoinverse of J
% pseudo_inv = pinv(J); % or from the slides: transpose(J)*(J*transpose(J))^-1;
% 
% % Calculate q_dot using the pseudoinverse
% q_dot = pseudo_inv * xi;
% 
% % Checking that we get the correct result
% xi - J*q_dot



% Velocites of the end effector to be imposed
v_0_4 = [0,-3,0].'; % [mm/s] 

% We only have the linear velocities, and of the angular velocities we dont
% care about the first two components, and we know that the last is:
% Ryx*omega_x-Rxx*omega_y = 0 from the appendix.
% We therefore built a reduced Jacobian matrix, with only what we need, so
% not 6x4 but 4x4.

% We will call the last element omega_star, is the only angular velocity we
% are interested in. This is 0 as given in the assignment
omega_star = 0; 
xi = [v_0_4; omega_star];

% Isolating the Jacobian we are interested in
J = J_EndEffector_angle2;

% Isolating the rotation matrix of the end effector - R
DH_parameters = [theta_1    a_1   0      pi/2
                 theta_2    0     a_2    0
                 theta_3    0     a_3    0
                 theta_4    0     a_4    0];
T_04 = DH(DH_parameters(1:4,:));
R = T_04(1:3,1:3);
Ryx = R(2,1); Rxx = R(1,1); % Elements of R we are interested in

% Creating a matrix to reduce J to a 4x4 Jacobian
M = [1   0   0   0    0   0
     0   1   0   0    0   0
     0   0   1   0    0   0
     0   0   0  Ryx -Rxx  0];

% Identify the values we are interested in and substitute
q_current = q(:,i).';
M = subs(M,[theta_1 theta_2 theta_3 theta_4],q_current);

% Computing the reduced Jacobian
J_red = M*J;

% Calculate q_dot using the inverse
q_dot = J_red\xi;

% Checking that we get the correct result
xi - J_red*q_dot



%% Problem 6: Trajectory planning

% First let's find the joint velocities using the Jacobian matrices from
% problem 5

% Given end effector velocities and accelerations - these are given 0 to 4,
% so wrt global frame, as they should be in order to find the q_dot
velocities = [0   0    0    0    0
              0  -27   0    27   0
              0   0   -27   0    0];

% Exctract xi, as I am not interested in omegas it is just the input vel.
xi = velocities; 

% Isolate the Jacobians we need and storing them in only one space
J = zeros([[3 4], length(angles)]);
J(:, :, 1) = J_EndEffector_angle1(1:3,:);
J(:, :, 2) = J_EndEffector_angle2(1:3,:);
J(:, :, 3) = J_EndEffector_angle3(1:3,:);
J(:, :, 4) = J_EndEffector_angle4(1:3,:);
J(:, :, 5) = J_EndEffector_angle5(1:3,:);

% Note: I have the positions as "q"

% Using a loop to compute inverse velocity, finding velocity of link q_dot
q_dot = zeros(4,length(angles));
for i=1:length(velocities)

    % Extracting current values
    xi_current = xi(:,i);
    J_current = J(:, :, i);

    % Performing the required rank test
    if rank(J_current)~=rank([J_current xi_current]) 
        disp('Rank test invalid, J cannot be inverted')
    end

    % Calculate the pseudoinverse of J
    pseudo_inv = pinv(J_current); % or from the slides: [transpose(J)*J]^-1*transpose(J)

    % Calculate q_dot using the pseudoinverse
    q_dot_current = pseudo_inv * xi_current;
    q_dot(:,i) = q_dot_current;

end

% Accelerations are given as null
q_ddot = zeros(4,5);

% We now have q, q_dot, q_ddot
 
% Initialize a-element matrices
A = zeros(6,4);  B = zeros(6,4);  
C = zeros(6,4);  D = zeros(6,4); 

% For each joint
% q      -> position of each joint for a certaing angle
% q_dot  -> velocity of each joint for a certaing angle
% q_ddot -> acceleration of each joint for a certaing angle

% Polinomia is: 
% position -> a_0 + a_1*t + a_2*t^2 + a_3*t^3 + a_4*t^4 + a_5*t^5
% velocity -> 0 + a_1 + 2*a_2*t + 3*a_3*t^2 + 4*a_4*t^3 + 5*a_5*t^4
% acceleration -> 0 + 0 + 2*a_2 + 6*a_3*t + 12*a_4*t^2 + 20*a_5*t^3

syms t
t_coefficients_syms = [1  t  t^2   t^3     t^4      t^5
                       0  1  2*t   3*t^2   4*t^3    5*t^4
                       0  0  2     6*t     12*t^2   20*t^3];
    
% Times go from 0 to 2 seconds
t_A = 2; t_B = 2; t_C = 2; t_D = 2; 
times = [t_A t_B t_C t_D];

% Loop per each time section - segment
for i=1:length(times)

    t_coefficients = [subs(t_coefficients_syms,t,0);subs(t_coefficients_syms,t,times(i))];

    % Loop per each element in q, so per each joint variable
    for j=1:height(q)

        % Find the vector of q variables, composed of pos, vel and acc at
        % initial position, vel and acc at final position, vel and acc.
        q_current = [q(j,i) q_dot(j,i) q_ddot(j,i) q(j,i+1) q_dot(j,i+1) q_ddot(j,i+1)].';
   
        % Solving the system to find the coefficients
        c_coefficients = t_coefficients\q_current;
    
        % Collecting the data
        if i==1
            A(:,j) = c_coefficients.';
        end
        if i==2
            B(:,j) = c_coefficients.';
        end
        if i==3
            C(:,j) = c_coefficients.';
        end
        if i==4
            D(:,j) = c_coefficients.';
        end

    end

end


%% Problem 7: Plotting obtained trajectory

% Creating the simbolic transfer function
syms theta_1 theta_2 theta_3 theta_4
DH_parameters = [theta_1    a_1   0      pi/2
                 theta_2    0     a_2    0
                 theta_3    0     a_3    0
                 theta_4    0     a_4    0];
T_04 = DH(DH_parameters);

% Position of end effector in simbolic
o_0_4 = T_04(1:3,4).';

% Wanted total number of points
points = 500;

o_0_4_array = [];
for i=1:4

    if i==1
        theta_1_current = A(:,1).'*[1 t t^2 t^3 t^4 t^5].'; theta_2_current = A(:,2).'*[1 t t^2 t^3 t^4 t^5].';
        theta_3_current = A(:,3).'*[1 t t^2 t^3 t^4 t^5].'; theta_4_current = A(:,4).'*[1 t t^2 t^3 t^4 t^5].';
    end
    if i==2
        theta_1_current = B(:,1).'*[1 t t^2 t^3 t^4 t^5].'; theta_2_current = B(:,2).'*[1 t t^2 t^3 t^4 t^5].';
        theta_3_current = B(:,3).'*[1 t t^2 t^3 t^4 t^5].'; theta_4_current = B(:,4).'*[1 t t^2 t^3 t^4 t^5].';
    end
    if i==3
        theta_1_current = C(:,1).'*[1 t t^2 t^3 t^4 t^5].'; theta_2_current = C(:,2).'*[1 t t^2 t^3 t^4 t^5].';
        theta_3_current = C(:,3).'*[1 t t^2 t^3 t^4 t^5].'; theta_4_current = C(:,4).'*[1 t t^2 t^3 t^4 t^5].';
    end
    if i==4
        theta_1_current = D(:,1).'*[1 t t^2 t^3 t^4 t^5].'; theta_2_current = D(:,2).'*[1 t t^2 t^3 t^4 t^5].';
        theta_3_current = D(:,3).'*[1 t t^2 t^3 t^4 t^5].'; theta_4_current = D(:,4).'*[1 t t^2 t^3 t^4 t^5].';
    end

    current_q = [theta_1_current theta_2_current theta_3_current theta_4_current].';

    o_0_4_current = subs(o_0_4,[theta_1 theta_2 theta_3 theta_4],current_q.');
    o_0_4_array = [o_0_4_array; o_0_4_current];
end  

t_values = linspace(0, 2-(2/(points/4)), points/4).';
o_0_4_array = double(subs(o_0_4_array,t,t_values));

% Plot the data
plot3(o_0_4_array(:,1),o_0_4_array(:,2),o_0_4_array(:,3))
% plot(o_0_4_array(:,2),o_0_4_array(:,3))
hold on



%% Plot the circle that we should obtain

% Coordinates of the circle center and ratio R - [mm]
p_0_c = [150 0 120];  
R = 32;   
x_04_3 = 0; % of x from 0 to 4, third element (sin Phi) - Horizontal

coordinates = zeros(37, 3);
for i=1:37

    % Building coordinate of current point
    phi = 2*pi/(36) * (i-1);
    coord = p_0_c + R*[0 cos(phi) sin(phi)];

    % Coordinates of wrist center and its rotation (phi) - all global
    x_c = coord(1); y_c = coord(2); z_c = coord(3);
    
    % Compute inverse kinematics with external function
    [theta_1, theta_2, theta_3, theta_4] = InverseKinAssignment(a_1,a_2,a_3,a_4,x_c,y_c,z_c,x_04_3);

    % Compute direct cinematics to find the coordinates of the end effector
    DH_parameters = [theta_1    a_1   0      pi/2
                     theta_2    0     a_2    0
                     theta_3    0     a_3    0
                     theta_4    0     a_4    0];
    T_04 = DH(DH_parameters);

    % Collecting the coordinates of end effector to plot them
    coordinates(i, :) = T_04(1:3,4);

end

plot3(coordinates(:,1),coordinates(:,2),coordinates(:,3))
    





% Save matrices A B C D in a mat file to use it in problem 10
save('ABCD_input','A','B','C','D')


