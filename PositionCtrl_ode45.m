%Position control with Inverse dynamic controller
%First, run loadRobotmodel1.m to initiate robot model.

close all

%% Trajectory in joint space.
% 3rd order polynomial
tf = 2; % final simulation time in sec
q0 = startConfiguration; %homeconfiguration of robot defined in robot model
qdesired = [0.0 ;20.0; 0 ;-110.0; 0 ;-40.0; 90.0;]*pi/180;

dq = [0;0;0;0;0;0;0];
tau = [0;0;0;0;0;0;0;];

wayPoints = [q0(1) qdesired(1);q0(2) qdesired(2);q0(3) qdesired(3);q0(4) qdesired(4);...
            q0(5) qdesired(5); q0(6) qdesired(6);q0(7) qdesired(7)];
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
[t,x] = ode45(@(t,x) lbr14EoM(t,x,lbr14,qTr,dqTr,ddqTr),[0 tf],x0);
plot(t,x(:,1:7));
legend;
title('Simulation result for tracking in each joint');
figure
show(lbr14,x(end,1:7)');
title('Final Configuration of Robot');
%% show animated plot of the robot configurations during simulation
%it doesnt show in realtime, so wait and you will se the results! :)

% figure
% title('robot path during simulation');
% for ii = 2:size(t,1)
%  pause(t(ii)-t(ii-1));
%  show(lbr14,x(ii,1:7)');
% end
%% function
function dx = lbr14EoM(t,x,robot,qTr,dqTr,ddqTr)
KD = eye(7,7)*50;
KP = eye(7,7) *50;

timeIndex = uint16(ceil(t*100))+1;
qd = [qTr(1,timeIndex); qTr(2,timeIndex); qTr(3,timeIndex); qTr(4,timeIndex); qTr(5,timeIndex) ;qTr(6,timeIndex) ;qTr(7,timeIndex)];
dqd = [dqTr(1,timeIndex); dqTr(2,timeIndex); dqTr(3,timeIndex); dqTr(4,timeIndex); dqTr(5,timeIndex) ;dqTr(6,timeIndex) ;dqTr(7,timeIndex)];
ddqd =  [ddqTr(1,timeIndex); ddqTr(2,timeIndex); ddqTr(3,timeIndex); ddqTr(4,timeIndex); ddqTr(5,timeIndex) ;ddqTr(6,timeIndex) ;ddqTr(7,timeIndex)];

qtilda = qd - x(1:7);
dqtilda = dqd - x(8:14);
%u = Kp*qtilda - Kd*x(8:14) + robot.gravityTorque(x(1:7)); %PD controller

aq =ddqd+ KP*qtilda + KD*dqtilda ;
u = robot.massMatrix(x(1:7))*aq+ robot.velocityProduct(x(1:7),x(8:14)) + robot.gravityTorque(x(1:7)); %velocityProduct = C(q,dq)*dq

dx = zeros(14,1);
dx(1) = x(8);
dx(2) = x(9);
dx(3) = x(10);
dx(4) = x(11);
dx(5) = x(12);
dx(6) = x(13);
dx(7) = x(14);
dx(8:end) = forwardDynamics(robot,x(1:7),x(8:end),u,[]);

end