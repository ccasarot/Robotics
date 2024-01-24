% Christian Casarotto - s223302 - DTU

function result = translation(axis,parameter)
    syms x y z

    if axis == x
            result = [1  0  0  parameter
                      0  1  0  0
                      0  0  1  0
                      0  0  0  1];
    end

    if axis == y
            result = [1  0  0  0
                      0  1  0  parameter
                      0  0  1  0
                      0  0  0  1];
    end 

    if axis == z
            result = [1  0  0  0
                      0  1  0  0
                      0  0  1  parameter
                      0  0  0  1];
    end


end


