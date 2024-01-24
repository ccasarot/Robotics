% Project Assignment: Educational Robotic Arm

clc; clear all

% Robot dimensions [mm]
a_1 = 50; a_2 = 93; a_3 = 93; a_4 = 50;



%% Problem 1: Direct kinematics

% Define free parameters of the robot (all rotations around z)
syms theta_1 theta_2 theta_3 theta_4

% By looking at the figure, define the Denavit-Hartenberg paramters for the
% robot with the given axis system:
DH_parameters = [theta_1       50  0   pi/2
                 theta_2+pi/2  0   93    0
                 theta_3       0   93    0
                 theta_4       0   50    0];

% Find transformations matrix from frame 0 to 4:
T_04 = DH(DH_parameters)

% Transformation matrix from frame 4 to 5 is only translations (no DH convention)
T_45 = [1 0 0 -15
        0 1 0  45
        0 0 1  0
        0 0 0  1];

% Overall transformation matrix from frame 0 to 5:
T_05 = T_04*T_45;




% From now on, we stop using the +pi/2 




%% Problem 2: Inverse kinematics

% Coordinates of end effector and its rotation (phi) - all global
x_c = 200;
y_c = 0;
z_c = 150;
x_04_3 = 0; % of x from 0 to 4, third element (sin Phi)

% Compute inverse kinematics with external function
[theta_1, theta_2, theta_3, theta_4] = InverseKinAssignment(a_1,a_2,a_3,a_4,x_c,y_c,z_c,x_04_3);

% Display obtained parameters
values_rad = [theta_1; theta_2; theta_3; theta_4];
values_degree = values_rad*(180/pi);
q = ['theta_1'; 'theta_2';'theta_3';'theta_4'];
T = table(q,values_rad,values_degree)

% Verify that we effectively get the correct result by going backwards
% Why does it not work using the +pi/2 ? 
DH_parameters = [theta_1    a_1  0      pi/2
                 theta_2    0    a_2    0
                 theta_3    0    a_3    0
                 theta_4    0    a_4    0];

% Find transformations matrix from frame 0 to 4:
T_04 = DH(DH_parameters)

% Plotting the robot
PlotRobot(a_1,a_2,a_3,a_4,theta_1,theta_2,theta_3,theta_4)

theta_1 = rad2deg(theta_1);
theta_2 = rad2deg(theta_2);
theta_3 = rad2deg(theta_3);
theta_4 = rad2deg(theta_4);

theta_1_robot = theta_1+150;
theta_2_robot = theta_2+60;
theta_3_robot = theta_3+60;
theta_4_robot = theta_4+60;





%% Problem 3: Find sequence of 37 robot configurations

% Coordinates of the circle center and ratio R - [mm]
p_0_c = [150 0 120];  
R = 32;   
x_04_3 = 0; % of x from 0 to 4, third element (sin Phi) - Horizontal

% Initialize q
q = zeros(37, 4);
coordinates = zeros(37, 3);

for i=1:37

    % Building coordinate of current point
    phi = 2*pi/(36) * (i-1);
    coord = p_0_c + R*[0 cos(phi) sin(phi)];

    % Coordinates of wrist center and its rotation (phi) - all global
    x_c = coord(1); y_c = coord(2); z_c = coord(3);
    
    % Compute inverse kinematics with external function
    [theta_1, theta_2, theta_3, theta_4] = InverseKinAssignment(a_1,a_2,a_3,a_4,x_c,y_c,z_c,x_04_3);

    % Collecting the current coordinate
    q_current = [theta_1 theta_2 theta_3 theta_4];
    q(i, :) = q_current;

    % Compute direct cinematics to find the coordinates of the end effector
    DH_parameters = [theta_1    a_1   0      pi/2
                     theta_2    0     a_2    0
                     theta_3    0     a_3    0
                     theta_4    0     a_4    0];
    T_04 = DH(DH_parameters);

    % Collecting the coordinates of end effector to plot them
    coordinates(i, :) = T_04(1:3,4);

    if i>1
        plot3(coordinates(1:i,1),coordinates(1:i,2),coordinates(1:i,3))
        hold on
    end

    % Plot the robot in the current configuration
    PlotRobot(a_1,a_2,a_3,a_4,theta_1,theta_2,theta_3,theta_4)
    drawnow; % To show the plot in real time
    
    % Condition to keep the robot at the end
    hold off
    if i==37
        hold on
    end

end

% Plot the coordinates (each row one set of coordinates)
q

% % Split the coordinates and plot
% coord_x = coordinates(:,1); coord_y = coordinates(:,2); coord_z = coordinates(:,3);
% plot3(coord_x,coord_y,coord_z)
% hold on


