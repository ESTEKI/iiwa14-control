%Position control with Inverse dynamic controller
%First, run loadRobotmodel1.m 
% desired position = qd = [0; 20*pi/180.0; 0; 45*pi/180.0; 0 ;0 ;0];


%% Trajectory in joint space.
% 3rd order polynomial
q0 = startConfiguration;
qdesired = [0.0 ;20.0; 0 ;-110.0; 0 ;-40.0; 90.0;]*pi/180;

dq = [0;0;0;0;0;0;0];
tau = [0;0;0;0;0;0;0;];

wayPoints = [q0(1) qdesired(1);q0(2) qdesired(2);q0(3) qdesired(3);q0(4) qdesired(4);...
            q0(5) qdesired(5); q0(6) qdesired(6);q0(7) qdesired(7)];
timePoints =  [ 0 2];
tvec = 0:0.01:2 ;
[qTr, qdTr, qddTr, ppTr] = cubicpolytraj(wayPoints, timePoints, tvec);
% ppTr or pp — Piecewise-polynomial
% it is an structure including coefficients of the polynomial
% pp.coeff contains rows in each decending (t^3...t^0) coefficients of polynomial
% note that it should be 7 rows in our case but it is 21. Looking at the
% pp.breaks which is [-1 0 2 3] it generates 3 set of polynomials for each
% time interval [-1 to 0 , 0 to 2, 2 to 3]; only time 0s to 2s is needed (rows 8
% to 14)
%pos = 0.0436332312998582*tvec.^3+-0.130899693899575*tvec.^2+0.523598775598299;
plot(tvec, qTr)
hold all
plot(timePoints, wayPoints, 'x')
xlabel('t')
ylabel('Positions')
legend('X-positions','time');
hold off
figure;
%pos = 0.0436332312998582*tvec.^3+-0.130899693899575*tvec.^2+	0.523598775598299;
%lbr14.geometricJacobian(q0,'iiwa_link_7')
plot(tvec,pos)
%% ODE simulation
% comented temporarily [t,x] = ode45(@(t,x) lbr14EoM(t,x,lbr14),[0 2],x0);
plot(t,x(:,1),'b',t,x(:,2),'r',t,x(:,3),'g',t,x(:,4),'y');
%%
% load saved data ' x values ...' and run this section to chech the results
plot(t,x(:,1),'b',t,x(:,2),'r',t,x(:,3),'g',t,x(:,4),'y');

%%lbr14.gravityTorque()
%%lbr14.homeConfiguration
%% function
function dx = lbr14EoM(t,x,robot)
Kd = eye(7,7)*50;
Kp = eye(7,7) *50;
qd = [0; 20*pi/180.0; 0; 45*pi/180.0; 0 ;0 ;0];

qtilda = qd - x(1:7);
%u = Kp*qtilda - Kd*x(8:14) + robot.gravityTorque(x(1:7)); PD
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