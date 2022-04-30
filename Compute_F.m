function out = Compute_F(in)

x1 = in(1); y1 = in(2); z1 = in(3);
x2 = in(28); y2 = in(29); z2 = in(30);
x3 = in(55); y3 = in(56); z3 = in(57);

Pf=[(x1+x2+x3)/3;...
     (y1+y2+y3)/3;...
     (z1+z2+z3)/3;...
     atan2((2*z1/3-z2/3-z3/3),(2*y1/3-y2/3-y3/3)); ...
     -atan2((2*z1/3-z2/3-z3/3),(2*x1/3-x2/3-x3/3)); ...
     atan2((2*y1/3-y2/3-y3/3),(2*x1/3-x2/3-x3/3))];
 
 pf = sqrt((x1-x2)^2+(y1-y2)^2+(z1-z2)^2);
 qf = sqrt((x1-x3)^2+(y1-y3)^2+(z1-z3)^2);
 rf = sqrt((x2-x3)^2 +(y2-y3)^2 +(z2-z3)^2);
 betaf =  acos((pf^2+qf^2-rf^2)/(2*pf*qf));
 Sf = [pf; qf; betaf];
 
 out = [Pf;Sf];
