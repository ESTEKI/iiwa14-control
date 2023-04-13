%worked!
dt= 0.05;
q0 = startConfiguration;
dq = [0;0;0;0;0;0;0];
tau = [0;0;0;0;0;0;0;];

x0 = [q0;dq];
[t,x] = ode78(@(t,x) lbr14EoM(t,x,lbr14),[0 2],x0);
plot(t,x(:,1),'b',t,x(:,2),'r',t,x(:,3),'g',t,x(:,4),'y');
%%
% load saved data ' x values ...'
plot(t,x(:,1),'b',t,x(:,2),'r',t,x(:,3),'g',t,x(:,4),'y');

lbr14.gravityTorque()
lbr14.homeConfiguration
%% function
function dx = lbr14EoM(t,x,robot)
Kd = eye(7,7)*50;
Kp = eye(7,7) *100;
qd = [0; 20*pi/180.0; 0; 45*pi/180.0; 0 ;0 ;0];

qtilda = qd - x(1:7);
u = Kp*qtilda - Kd*x(8:14) + robot.gravityTorque(x(1:7));
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