function [qTr, dqTr, ddqTr, ppTr,tvec] = jointSpaceTrajectory(startConfig,endConfig,t0,tf)
% Trajectory in joint space.
% 3rd order polynomial
q0 = startConfig; %homeconfiguration of robot defined in robot model
qdesired = endConfig;

wayPoints = [q0(1) qdesired(1);q0(2) qdesired(2);q0(3) qdesired(3);q0(4) qdesired(4);...
            q0(5) qdesired(5); q0(6) qdesired(6);q0(7) qdesired(7)];
timePoints =  [ t0 tf];
tvec = t0:0.01:tf ;
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

end