syms xx yy zz xy xz yz 

I = [xx xy xz; xy yy yz ; xz yz zz];

Ip = R*I*R'
% 
% Ip =
%  
% [ xx,  xy, -xz]
% [ xy,  yy, -yz]
% [-xz, -yz,  zz]