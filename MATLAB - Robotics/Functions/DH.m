% Christian Casarotto - s223302 - DTU

% Takes as input a matrix with Denavit Hartenberg parameters (each row is a
% set of paramters: theta, d, a and alpha, corresponding to passing from
% one reference phrame of the robot to the other.

% Gives as output the T matrix descriving the frame rotation/translation
% from initial to final frame, following Denavit Hartenberg convention

% Note: works independentely of how many rows are in matrix

function result = DH(matrix)
syms x y z

    for i = 1:size(matrix,1) % Loop for each row
        % Define DH parameters for i row
        theta = matrix(i,1);
        d = matrix(i,2);
        a = matrix(i,3);
        alpha = matrix(i,4);
    
        % Create the 4 matrices for current i row
        T_1 = rotation(z,theta);
        T_2 = translation(z,d);
        T_3 = translation(x,a);
        T_4 = rotation(x,alpha);
        
        % Compute T matrix for that row and collect it
        T{i} = T_1*T_2*T_3*T_4;
    end
        
    % Find final matrix by product of each T matrix
    result = T{1};
        for i = 2:length(T) % Loop starts from 2nd
            result = result * T{i};
        end
        
end
