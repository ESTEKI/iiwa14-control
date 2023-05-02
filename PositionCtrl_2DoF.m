%Position control of 2-link planar manipulator with PD,Inverse dynamic,
%Robust Inv. Dyn.,Adaptive ID, Passivity-based, Adaptive Passivity-based controller

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
KP = eye(2,2) *40;

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

%--PD--- 
%u = KP*qtilda - KD*x(3:4) + G; %PD controller

%Inv Dyn
 aq = ddqd + KP*qtilda + KD*dqtilda ;
 u = M*aq+ C*x(3:4) + G; %velocityProduct = C(q,dq)*dq

% Robust Inv. Dyn.
  
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
  u = M*aq+ C*x(3:4) + G; 

% Passivity-based controller-----------
%  landa = eye(7,7).* 10;
%  K = KP;
%  v = dqd-landa*qtilda;
%  a = ddqd - landa*dqtilda;
%  r = dqtilda + landa*qtilda;
%  % note that velocityProduct = C(q,dq)*dq so, C = C(q,dq)*dq*pinv(dq). It
%  % works but not recomended. Also takes a long time for ODE to solve.
%  u = robot.massMatrix(x(1:7))*a + robot.velocityProduct(x(1:7),x(8:14))*pinv(x(8:14))*v + robot.gravityTorque(x(1:7)) + K*r; 

 



    


dx(3:4) = inv(M)*(u-C*x(3:4)-G);

end