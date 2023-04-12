dt= 0.05;
q0 = startConfiguration;
dq = [0;0;0;0;0;0;0];
tau = [0;0;0;0;0;0;0;];

x0 = [q0;dq];
[t,x] = ode78(@(t,x) lbr14EoM(t,x,lbr14),[0 2],x0);
plot(t,x(:,1),'b',t,x(:,2),'r',t,x(:,3),'g',t,x(:,4),'y');


%%
% 
% for step = 1:size(t,1)
%    show(lbr14,x(step,1:7)','visuals','off');
%    %pause(0.05);
% end
% for step = 0:dt:5.0
% ddq = forwardDynamics(lbr14,q,dq,tau);
% dq = ddq*dt + dq;
% q = dq*dt+ q;
% % check the bounderies of each joint, if violated set it to the boundery
% % element
% loc = find(q<-jointPosLimitsUpperBound); 
% q(loc) = -jointPosLimitsUpperBound(loc);
% dq(loc) = 0;
% ddq(loc) = 0;
% loc = find(q>jointPosLimitsUpperBound); 
% q(loc) = jointPosLimitsUpperBound(loc);
% dq(loc) = 0;
% ddq(loc) = 0;
% 
% 
% 
% show(lbr14,q);
% pause(dt*1);
% end
lbr14.gravityTorque()
lbr14.homeConfiguration
%% functions 
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
% loc = find(x(1:7)<-jointPosLimitsUpperBound); 
%x(loc) = -jointPosLimitsUpperBound(loc);

% dq(loc) = 0;
% ddq(loc) = 0;
end