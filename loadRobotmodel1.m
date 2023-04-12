%%
lbr14 = loadrobot("kukaIiwa14");
lbr14.DataFormat= "column";
lbr14.Gravity = [0 0 -9.81];
% All parameters extracted from the following paper :'Parameter identification of the
%KUKA LBR iiwa robot including
%constraints on physical feasibility' Stürz, Yvonne R.; Affolter, Lukas M.;
%Smith, Roy 2017
links = lbr14.Bodies;
links{1,2}.Mass = 3.94781; %body index for the 1ts link is 2
links{1,3}.Mass = 4.50275;
links{1,4}.Mass = 2.45520;
links{1,5}.Mass = 2.61155;
links{1,6}.Mass = 3.410;
links{1,7}.Mass = 3.38795;
links{1,1}.Mass = 0.35432;
R = [-1 0 0;0 -1 0; 0 0 1]; %joint frames in the paper are not identical to those in matlab model 
 % only links 1,3,4,7 are different
lC1 = [-0.00351 0.00160 -0.03139] *R; %lCix lCiy lCiz for the first link
lC2 = [-0.00767 -0.16669 -0.00355];
lC3 = [-0.00225 -0.03492 -0.02652]*R;
lC4 = [-0.05268 0.03818 0.03844]*R;
lC5 = [-0.00237 -0.21134 0.00277];
lC6 = [0.02019 -0.02750 0.0005];
lC7 = [-0.02324 0.07138 0.00795]*R;


%%
links{1,2}.CenterOfMass = lC1;
links{1,3}.CenterOfMass = lC2;
links{1,4}.CenterOfMass = lC3;
links{1,5}.CenterOfMass = lC4;
links{1,6}.CenterOfMass = lC5;
links{1,7}.CenterOfMass = lC6;
links{1,8}.CenterOfMass = lC7;
%%
% [Ixx Iyy Izz Iyz Ixz Ixy] vector

 links{1,2}.Inertia = [0.00455  0.00454 0.00029 0.00001 0.0 0.0];
links{1,3}.Inertia = [0.00032 0.0001 0.00042 0 0 0];
links{1,4}.Inertia = [0.00223 0.00219 0.00073 -0.00007 -0.00007 0];
links{1,5}.Inertia = [0.03844 0.01144 0.4988 +0.00111 +0.00112 0.00088];
links{1,6}.Inertia = [0.00277 0.00284 0.00012 0 0 0];
links{1,7}.Inertia = [0.0005 0.00281 0.00232 0 0 0];
links{1,8}.Inertia = [0.00795 0.01089 0.00294 +0.0000  +0.0000 0.000];
%Real values in paper:
% links{1,2}.Inertia = [0.00455  0.00454 0.00029 0.00001 0.0 0.0];
% links{1,3}.Inertia = [0.00032 0.0001 0.00042 0 0 0];
% links{1,4}.Inertia = [0.00223 0.00219 0.00073 0.00007 0.00007 0];
% links{1,5}.Inertia = [0.03844 0.01144 0.4988 -0.00111 -0.00112 0.00088];
% links{1,6}.Inertia = [0.00277 0.00284 0.00012 0 0 0];
% links{1,7}.Inertia = [0.0005 0.00281 0.00232 0 0 0];
% links{1,8}.Inertia = [0.00795 0.01089 0.00294 -0.00029  -0.00029 0.00029];

%%
startConfiguration = [0; 30*pi/180.0; 0; 45*pi/180.0; 0 ;0 ;0];
links{1,3}.Joint.HomePosition = startConfiguration(2);
links{1,5}.Joint.HomePosition = startConfiguration(4);
lbr14.gravityTorque()
%%
for nj = 2:8
jointPosLimits(nj-1,:) = links{1,nj}.Joint.PositionLimits;
end
jointPosLimitsUpperBound(:,1) = (jointPosLimits(:,2));

  
%%
show(lbr14,'visuals','off')