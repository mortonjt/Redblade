


function [xp,yp] = rotmat(x,y,theta)


xp = x*cos(theta)-y*sin(theta);
yp = x*sin(theta)+y*cos(theta);


end