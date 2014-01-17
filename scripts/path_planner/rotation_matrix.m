function [ rotated ] = rotation_matrix(rotate_me, theta)
    x = rotate_me(1);
    y = rotate_me(2);
    
    rotated(1) = x*cos(theta) - y*sin(theta);
    rotated(2) = x*sin(theta) + y*cos(theta);

end

