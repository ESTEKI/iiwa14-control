syms xx yy zz xy xz yz 

I = [xx xy xz; xy yy yz ; xz yz zz];

Ip = R*I*R'  % from paper to matlab kuka model
% output:
% Ip =
%  
% [ xx,  xy, -xz]
% [ xy,  yy, -yz]
% [-xz, -yz,  zz]

% as you can see all the values on main diagonal remain the same 
