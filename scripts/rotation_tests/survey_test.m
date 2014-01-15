
field_width1 = 3.5;
field_width2 = 7;

p1.x = 4.1;
p1.y = 2.2;
p2.x = -1.1;
p2.y = 6.2;

theta = atan2((p2.y-p1.y),(p2.x-p1.x));
ab_dist = sqrt((p2.y-p1.y)*(p2.y-p1.y)+(p2.x-p1.x)*(p2.x-p1.x));

[rp1.x,rp1.y] = rotmat(p1.x,p1.y,-theta);
[rp2.x,rp2.y] = rotmat(p2.x,p2.y,-theta);

rp3.x = rp1.x;
rp3.y = rp1.y + field_width1;
rp4.x = rp2.x;
rp4.y = rp2.y + field_width1;

[p3.x,p3.y] = rotmat(rp3.x,rp3.y,theta);
[p4.x,p4.y] = rotmat(rp4.x,rp4.y,theta);

rp5.x = rp1.x;
rp5.y = rp1.y + field_width2;
rp6.x = rp2.x;
rp6.y = rp2.y + field_width2;

[p5.x,p5.y] = rotmat(rp5.x,rp5.y,theta);
[p6.x,p6.y] = rotmat(rp6.x,rp6.y,theta);

figure(1); clf; hold on;
axis([-10 10 -10 10]);
% 
% plot(test1x,test1y,'m*');
% plot(test2x,test2y,'m*');

plot(p1.x,p1.y,'b*');
plot(p2.x,p2.y,'b*');

plot(p3.x,p3.y,'go');
plot(p4.x,p4.y,'go');

plot(p5.x,p5.y,'r*');
plot(p6.x,p6.y,'r*');

