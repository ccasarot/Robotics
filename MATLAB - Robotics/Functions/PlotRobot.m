% Christian Casarotto - s223302 - DTU

% Takes as input a list of dimensione of the four robot links and a list of
% angles corresponding to these links

% Gives as output the plot of the robot in the given configuration, where 
% each element has a different color

% Note: works ONLY for robots with 4 elements, having 4 revolut joints 
%       whose free coordinates are four angles, where the first one is the
%       angle of rotation arounf Z, and the other three rotations around Y.
%       Here we are talking about global axes, where looking at the 3D
%       space Z points up, X to the right, and Y enters the plane.

function PlotRobot(a_1,a_2,a_3,a_4,theta_1,theta_2,theta_3,theta_4)

    % Find intermediate positions in the plane
    pos_0 = [0 0 0];
    pos_1 = [0 0 a_1];
    pos_2 = pos_1 + [a_2*cos(theta_2)*cos(theta_1)                 a_2*cos(theta_2)*sin(theta_1)                 a_2*sin(theta_2)];
    pos_3 = pos_2 + [a_3*cos(theta_2+theta_3)*cos(theta_1)         a_3*cos(theta_2+theta_3)*sin(theta_1)         a_3*sin(theta_2+theta_3)];
    pos_4 = pos_3 + [a_4*cos(theta_2+theta_3+theta_4)*cos(theta_1) a_4*cos(theta_2+theta_3+theta_4)*sin(theta_1) a_4*sin(theta_2+theta_3+theta_4)];
    
    % Group up coordinates 
    coord = [pos_0; pos_1; pos_2; pos_3; pos_4];
    
    % Plot each element of a different color with a loop
    colors = ["black" "red" "blue" "green"];
    for i=1:4
    coord_x = coord(i:i+1,1);
    coord_y = coord(i:i+1,2);
    coord_z = coord(i:i+1,3);
        plot3(coord_x.',coord_y.',coord_z.','-o','Color',colors(i))
        hold on
    end
        grid on
        xlim([-250, 250]); 
        ylim([-250, 250]); 
        zlim([0, 400]); 
        
end
