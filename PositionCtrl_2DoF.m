%Position control of 2-link planar manipulator with PD,Inverse dynamic,
%Robust Inv. Dyn.,Adaptive ID, Passivity-based, Robust Passivity-based controller

% To test one controller, comment out others control input 'u' calculation
% or the whole controller. So, the last uncommented u is the input to
% simulation (currently, RPBC).
close all

%% Trajectory in joint space.
% 3rd order polynomial
tf = 2; % final simulation time in sec
q0 = [0;0]; %homeconfiguration of robot defined in robot model
qdesired = [10.0 ;20.0]*pi/180;

dq = [0;0;];
tau = [0;0;];

wayPoints = [q0(1) qdesired(1);q0(2) qdesired(2);];
timePoints =  [ 0 2];
tvec = 0:0.01:2 ;
[qTr, dqTr, ddqTr, ppTr] = cubicpolytraj(wayPoints, timePoints, tvec);
% ppTr or pp — Piecewise-polynomial
% it is an structure including coefficients of the polynomial
% pp.coeff contains rows in each decending (t^3...t^0) coefficients of polynomial
% note that it should be 7 rows in our case but it is 21. Looking at the
% pp.breaks which is [-1 0 2 3] it generates 3 set of polynomials for each
% time interval [-1 to 0 , 0 to 2, 2 to 3]; only time 0s to 2s is needed (rows 8
% to 14)
%position  = 0.0436332312998582*tvec.^3+-0.130899693899575*tvec.^2+0.523598775598299; 
plot(tvec, qTr)
xlabel('t')
ylabel('Positions')
legend;
title('Desired Trajectory for each joint');
hold off
figure;
%% ODE simulation

x0 = [q0;dq];
KD = eye(2,2)*10;
KP = eye(2,2) *40;
[P_2link,B] = P_calculateFunc(KP,KD,10);
[t,x] = ode45(@(t,x) robot2DoF(t,x,qTr,dqTr,ddqTr,P_2link),[0 tf],x0);
plot(t,x(:,1:2));
legend;
title('Simulation result for tracking in each joint');


%% function
function dx = robot2DoF(t,x,qTr,dqTr,ddqTr,P2link)
dx = zeros(4,1);
m1 = 1;
m2=1;
l1=1;
l2=1;
lc1=0.5;
lc2=0.5;
g=9.8;
I1=2;
I2=2;



KD = eye(2,2)*10;
KP = eye(2,2) *100;

timeIndex = uint16(ceil(t*100))+1;
% qdesired dqdesired ...
qd = [qTr(1,timeIndex); qTr(2,timeIndex); ];
dqd = [dqTr(1,timeIndex); dqTr(2,timeIndex);];
ddqd =  [ddqTr(1,timeIndex); ddqTr(2,timeIndex);];

qtilda = qd - x(1:2);
dqtilda = dqd - x(3:4);

dx(1) = x(3);
dx(2) = x(4);


d11 = (m1*lc1^2+m2*(l1^2+l2^2+2*l1*lc2*cos(x(2)))+I1+I2);
d12=(m2*(lc2^2+l1*lc2*cos(x(2)))+I2);
d21=d12;
d22=(m2*l1*lc2^2+I2);
h = -(m2*l1*lc2*sin(x(2)));
g1=(m1*lc1+m2*l1)*g*cos(x(1))+m2*lc2*g*cos(x(1)+x(2));
g2= m2*lc2*g*cos(x(1)+x(2));
M = [d11 d12; d21 d22];
C = [h*x(2) h*x(2)+h*x(1);-h*x(1) 0];
G = [g1;g2];

%--PD--------------------------
%u = KP*qtilda - KD*x(3:4) + G; %PD controller

%Inv Dyn ---------------------------------
 aq = ddqd + KP*qtilda + KD*dqtilda ;
 u = M*aq+ C*x(3:4) + G; %velocityProduct = C(q,dq)*dq

% Robust Inv. Dyn. -------------
  ro = 0.2;
  epsilon = 0.02;
  error = [qtilda; dqtilda];
  B = [zeros(2,2); eye(2,2)] ;
  BPe = B'*P2link*error;
  normBPe = norm(BPe);

  if normBPe > epsilon
    da = -ro.*BPe/normBPe;
  else
    da = -ro/ epsilon * BPe;
  end
  aq = ddqd+ KP*qtilda + KD*dqtilda + da ;
  
  m1 = 0.8;
  m2=0.8;
  l1=0.8;
  l2=0.8;
  lc1=0.4;
  lc2=0.4;
  I1=2;
  I2=2;

  d11 = (m1*lc1^2+m2*(l1^2+l2^2+2*l1*lc2*cos(x(2)))+I1+I2);
  d12=(m2*(lc2^2+l1*lc2*cos(x(2)))+I2);
  d21=d12;
  d22=(m2*l1*lc2^2+I2);
  h = -(m2*l1*lc2*sin(x(2)));
  g1=(m1*lc1+m2*l1)*g*cos(x(1))+m2*lc2*g*cos(x(1)+x(2));
  g2= m2*lc2*g*cos(x(1)+x(2));
  Mh = [d11 d12; d21 d22];
  Ch = [h*x(2) h*x(2)+h*x(1);-h*x(1) 0];
  Gh = [g1;g2];
  u = Mh*aq+ Ch*x(3:4) + Gh;



% Adaptive Inverse Dynamic---------------------
 gama = eye(5,5)*50;
 persistent dx_p;
 if (isempty(dx_p))
     dx_p = dx;
 end
 
 teta0 = [m1*lc1^2+m2*(l1^2+lc2^2)+I1+I2; m2*l1*lc2; m2*lc2^2+I2; m1*lc1+m2*l1; m2*l2];
 Y = [dx_p(3) cos(x(2))*(2*dx_p(1)+dx_p(2))-sin(x(2))*(x(3)^2+2*x(3)*x(4)) dx_p(4) g*cos(x(1))  g*cos(x(1)+x(2));
     0 cos(x(2))*dx_p(3)+sin(x(2))*x(3)^2 dx_p(3)+dx_p(4) 0 g*cos(x(1)+x(2))]; %using dx_p as the previus step acceleration
 
 phi = inv(Mh)* Y;

 persistent t0; % we need time step of ode45 to calculate theta hat dot
 persistent theta_hat;
 persistent theta_dot_old;

 if (isempty(t0))
    t0 = 0;
 end
 if (isempty(theta_hat))
    theta_hat = zeros (5,1);
 end
 if (isempty(theta_dot_old))
    theta_dot_old = zeros (5,1);
 end
 dt = t - t0;
 t0 =t;
 theta_dot = -inv(gama)*phi'*B'*P2link*error;
 theta_hat = theta_hat + (dt/2)* (theta_dot + theta_dot_old);%Integrator -inv(gama)*
 theta_dot_old = theta_dot;
aq = ddqd + KP*qtilda + KD*dqtilda;
u = Mh*aq+ Ch*x(3:4) + Gh + inv(Mh)*Y*(teta0-theta_hat);


% Passivity-based controller-----------
 landa = eye(2,2).* 10;
 K = KP;
 v = dqd-landa*qtilda;
 a = ddqd - landa*dqtilda;
 r = dqtilda + landa*qtilda;
%  u = M*a + C*v + G + K*r; 

% Passivity-based Robust controller-----------
 %Parameters for Theta matrix

Y = [a(1) cos(x(2))*(2*a(1)+a(2))-sin(x(2))*(v(1)^2+2*v(1)*v(2)) a(2) g*cos(x(1))  g*cos(x(1)+x(2));
    0 cos(x(2))*a(1)+sin(x(2))*v(1)^2 a(1)+a(2) 0 g*cos(x(1)+x(2))];

teta0 = [m1*lc1^2+m2*(l1^2+lc2^2)+I1+I2; m2*l1*lc2; m2*lc2^2+I2; m1*lc1+m2*l1; m2*l2];
  Ytr = Y'*r;
  normYtr = norm(Ytr);

  if normYtr > epsilon
    dtheta = -ro*(Ytr/normBPe);
  else
    dtheta = -(ro/ epsilon) * Ytr;
  end
  %u = Y*(teta0 + dtheta)-K*r; %PBRC

% Passivity-based Adaptive controller-----------
 persistent theta_hat2;
 persistent theta_dot_old2; 
 
 if (isempty(theta_hat2))
    theta_hat2 = zeros (5,1);
 end
 if (isempty(theta_dot_old2))
    theta_dot_old2 = zeros (5,1);
 end
 
 theta_hat_dot2 = -inv(gama)*Y'*r;
 theta_hat2 = theta_hat2 + (dt/2)* (theta_hat_dot2 + theta_dot_old2);%Integrator 
 theta_dot_old2 = theta_hat_dot2;
  u = Y*(theta_hat2)+K*r; %PBAC

dx(3:4) = inv(M)*(u-C*x(3:4)-G);
dx_p = dx;
end