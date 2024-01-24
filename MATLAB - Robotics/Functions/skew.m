% Christian Casarotto - s223302 - DTU

function result = skew(vector)

    n = length(vector);  

    if n ~= 3
        disp('Length must be 3')
        result = 0;
    end

    if n == 3
       result = [0          -vector(3)   vector(2);
                 vector(3)       0      -vector(1);
                 vector(2)  vector(1)           0];
       end
end