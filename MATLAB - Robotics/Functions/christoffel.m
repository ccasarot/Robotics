% Christian Casarotto - s223302 - DTU

% Takes as input the Inertia matrix D and vector of free coordinates q

% Gives as output the christoffel parameters for that combination of D, q

function christoffel_parameters = christoffel(D,q)
    
    % Initialize
    syms christoffel_parameters

    % Loop for each index. Depending on the length of q you get a number of
    % c_ijk parameters
    for i=1:length(q)
        for j=1:length(q)
            for k=1:length(q)
                christoffel_parameters(i,j,k) = 0.5*(diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)));
            end
        end
    end      

end


