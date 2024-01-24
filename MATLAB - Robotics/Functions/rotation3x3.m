% Christian Casarotto - s223302 - DTU

function result = rotation3x3(axis,angle)
    syms x y z
    
    if axis == x
        result = [1         0          0         
                  0     cos(angle)   -sin(angle) 
                  0     sin(angle)   cos(angle)];
    end

    if axis == y
        result = [cos(angle)   0   sin(angle) 
                     0         1       0     
                 -sin(angle)   0   cos(angle)];
    end

    if axis == z
        result = [cos(angle)   -sin(angle)   0
                  sin(angle)   cos(angle)    0
                      0            0         1];
    end


end


