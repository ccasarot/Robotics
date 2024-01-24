% Project Assignment: Educational Robotic Arm

clc; clear all
syms x y z

% Robot dimensions [mm]
a_1 = 50;  a_2 = 93;  a_3 = 93;  a_4 = 50;



%% Preliminary: find the Jacobian in symbolic

syms theta_1 theta_2 theta_3 theta_4

% Updating DH parameters - pi/2 ???
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

% Creating J matrix
J_syms = [cross(z_0,(o_4-o_0))  cross(z_1,(o_4-o_1))  cross(z_2,(o_4-o_2))  cross(z_3,(o_4-o_3))
                  z_0                   z_1                   z_2                   z_3         ];



%% Problem 8: Find singularities

% From Problem 3: Find sequence of 37 robot configurations

% Coordinates of the circle center and ratio R - [mm]
p_0_c = [150 0 120];  
R = 32;   
x_04_3 = 0; % of x from 0 to 4, third element (sin Phi) - Horizontal

condition = [];
for i=1:37

    % Building coordinate of current point
    phi = 2*pi/(36) * (i-1);
    coord = p_0_c + R*[0 cos(phi) sin(phi)];

    % Coordinates of wrist center and its rotation (phi) - all global
    x_c = coord(1); y_c = coord(2); z_c = coord(3);
    
    % Compute inverse kinematics with external function
    [theta_1_current, theta_2_current, theta_3_current, theta_4_current] = InverseKinAssignment(a_1,a_2,a_3,a_4,x_c,y_c,z_c,x_04_3);

    % Collecting the current coordinate
    q_current = [theta_1_current theta_2_current theta_3_current theta_4_current];

    % Find current J matrix by substitution
    J = double(subs(J_syms, [theta_1 theta_2 theta_3 theta_4], q_current));

    % Collect current value of condition of the matrix
    condition(i) = cond(J);

end

% Plotting
% AngularPosition = linspace(-pi, pi, 37);
AngularPosition = linspace(0, 2*pi, 37);

figure;
plot(AngularPosition, condition, 'b', 'LineWidth', 2); 
title('Condition over Angular position - Trajectory from problem 3');
xlabel('Position [rad]');
xticks([0, pi/2, pi, 3*pi/2, 2*pi]);
xticklabels({'0', 'π/2', 'π', '3π/2', '2π'});
xlim([0, 2*pi]);
ylabel('Condition');
grid on;




% From problem 7, find the q vector of interpolated positions

% Let's first import the matrices from the .MAT file, to avoid calculating
% them again in this file
load('ABCD_input');

% Define the desired number of points for the interpolation:
points = 100;

% Interpolate to find values of position, using the loaded A B C D matrices
syms t
q = [];
for i=1:4
    if i==1
        q_1_current = A(:,1).'*[1 t t^2 t^3 t^4 t^5].'; q_2_current = A(:,2).'*[1 t t^2 t^3 t^4 t^5].';
        q_3_current = A(:,3).'*[1 t t^2 t^3 t^4 t^5].'; q_4_current = A(:,4).'*[1 t t^2 t^3 t^4 t^5].';
    end
    if i==2
        q_1_current = B(:,1).'*[1 t t^2 t^3 t^4 t^5].'; q_2_current = B(:,2).'*[1 t t^2 t^3 t^4 t^5].';
        q_3_current = B(:,3).'*[1 t t^2 t^3 t^4 t^5].'; q_4_current = B(:,4).'*[1 t t^2 t^3 t^4 t^5].';
    end
    if i==3
        q_1_current = C(:,1).'*[1 t t^2 t^3 t^4 t^5].'; q_2_current = C(:,2).'*[1 t t^2 t^3 t^4 t^5].';
        q_3_current = C(:,3).'*[1 t t^2 t^3 t^4 t^5].'; q_4_current = C(:,4).'*[1 t t^2 t^3 t^4 t^5].';
    end
    if i==4
        q_1_current = D(:,1).'*[1 t t^2 t^3 t^4 t^5].'; q_2_current = D(:,2).'*[1 t t^2 t^3 t^4 t^5].';
        q_3_current = D(:,3).'*[1 t t^2 t^3 t^4 t^5].'; q_4_current = D(:,4).'*[1 t t^2 t^3 t^4 t^5].';
    end
    q_current = [q_1_current q_2_current q_3_current q_4_current];
    q = [q; q_current];
end  

% "Populate" the q vector using how many points needed
t_values = linspace(0, 2-(2/(points/4)), points/4).';
q = double(subs(q,t,t_values));

condition = [];
for i=1:points

    % Find current J matrix by substitution
    J = double(subs(J_syms, [theta_1 theta_2 theta_3 theta_4], q(i,:)));

    % Collect current value of condition of the matrix
    condition(i) = cond(J);

end

% Plotting
AngularPosition = linspace(0, 2*pi, points);

figure;
plot(AngularPosition, condition, 'r', 'LineWidth', 2); 
title('Condition over Angular position - Trajectory from problem 6');
xlabel('Position [rad]');
xticks([0, pi/2, pi, 3*pi/2, 2*pi]);
xticklabels({'0', 'π/2', 'π', '3π/2', '2π'});
xlim([0, 2*pi]);
ylabel('Condition');
grid on;



%% Problem 9: Plotting static torque over position

% Force vector wrt global axes [N]
F = [0 0 -1 0 0 0].';

% Loop to compute the torque as array where each column is one torque
tau = [];
for i=1:points

    % Find jacobian matrix for current q combination
    J = double(subs(J_syms, [theta_1 theta_2 theta_3 theta_4], q(i,:)));

    % Compute current torque
    tau_current = transpose(J)*F;

    % Collect in an array the four values of tau per each iteration
    tau = [tau; tau_current.'];

end

% Plotting
AngularPosition = linspace(0, 2*pi, points);

% Plot each torque column
figure;
plot(AngularPosition, tau(:, 1), 'b', 'LineWidth', 2); hold on;
plot(AngularPosition, tau(:, 2), 'r', 'LineWidth', 2);
plot(AngularPosition, tau(:, 3), 'g', 'LineWidth', 2); 
plot(AngularPosition, tau(:, 4), 'm', 'LineWidth', 2);
title('Static torque evolution over Angular position');
xlabel('Position [rad]');
xticks([0, pi/2, pi, 3*pi/2, 2*pi]);
xticklabels({'0', 'π/2', 'π', '3π/2', '2π'});
xlim([0, 2*pi]);
ylabel('Torque [Nmm]');
legend('\tau_1', '\tau_2', '\tau_3', '\tau_4');
grid on;
hold off;


