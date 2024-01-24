% Christian Casarotto - s223302 - DTU

% Input:  christoffel_parameters and velocity vector q_dot. Note that the
%         dimensions of the two array must be coherent.
% Output: C matrix of the form [C(q,q_dot)]_kj -> note that the indeces are
%         swapped: christoffel_parameters = c_ijk while C = []_kj

function C = christoffelMatrix(christoffel_parameters,q_dot)
   
    syms C

    for k=1:length(q_dot)
        for j=1:length(q_dot)
            C(k,j) = 0; % Initially zero
            for i=1:length(q_dot)
                C(k,j) = C(k,j) + christoffel_parameters(i,j,k)*q_dot(i);
            end
        end
    end  

end


