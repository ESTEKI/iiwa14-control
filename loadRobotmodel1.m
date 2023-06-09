%%
lbr14 = loadrobot("kukaIiwa14");
lbr14.DataFormat= "column";
format long
lbr14.Gravity = [0 0 -9.81];
% All parameters extracted from the following paper :'Parameter identification of the
%KUKA LBR iiwa robot including constraints on physical feasibility' Stürz, Yvonne R.; Affolter, Lukas M.;
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
                            %links 1,3,4,7 are different so we multiply them by R to rotate frame

Rvrep12357 = R;             % frame transformations from paper to coppeliaSim
                            % again, frames are diffrent!
Rvrep4 = [-1 0 0; 0 0 -1; 0 -1 0];
Rvrep6 = [-1 0 0 ; 0 0 1; 0 1 0];

%Center of masses in paper:
lC1 = [-0.00351 0.00160 -0.03139] ; %lCix lCiy lCiz for the first link
lC2 = [-0.00767 -0.16669 -0.00355];
lC3 = [-0.00225 -0.03492 -0.02652];
lC4 = [0.0002 0.05268 0.03818];
lC5 = [0.00005 -0.00237 -0.21134];
lC6 = [0.00049 0.02019 -0.02750 ];
lC7 = [-0.03466 -0.02324 0.07138];


%% Center of Mass
links{1,2}.CenterOfMass = lC1*R;
links{1,3}.CenterOfMass = lC2;
links{1,4}.CenterOfMass = lC3*R;
links{1,5}.CenterOfMass = lC4*R;
links{1,6}.CenterOfMass = lC5;
links{1,7}.CenterOfMass = lC6;
links{1,8}.CenterOfMass = lC7*R;

%use below values to set KUKA lbr iiwa CoM parameters in CoppelliaSim
CoMvrep(1,:) = lC1*Rvrep12357;
CoMvrep(2,:) = lC2*Rvrep12357;
CoMvrep(3,:) = lC3*Rvrep12357;
CoMvrep(4,:) = lC4*Rvrep4;
CoMvrep(5,:) = lC5*Rvrep12357;
CoMvrep(6,:) = lC6*Rvrep6;
CoMvrep(7,:) = lC7*Rvrep12357;


%% Inertia
%Real values in paper:
I1 = [0.00455  0.00454 0.00029 0.00001 0.0 0.0];
I2 = [0.00032 0.0001 0.00042 0 0 0];
I3 = [0.00223 0.00219 0.00073 0.00007 0.00007 0];
I4 = [0.03844 0.01144 0.04988 -0.00111 -0.00112 0.00088];
I5 = [0.00277 0.00284 0.00012 0 0 0];
I6 = [0.0005 0.00281 0.00232 0 0 0];
I7 = [0.00795 0.01089 0.00294 -0.00029  -0.00029 0.00029];
%I4 = [0.03844 0.00088 -0.00112 ; 0.00088 0.01144 -0.00111; -0.00112 -0.00111 0.04988];

%values after changing the frame from those in paper to those in matlab's robot
%model definition (use show(lbr14) to chech frames)
%to do so, I_p = R * I * transpose(R). where I_p is the new inertia matrix
%refer to http://www.kwon3d.com/theory/moi/triten.html eq.4 and
%transformInertia.m code
% [Ixx Iyy Izz Iyz Ixz Ixy] vector for rigidbodytree in matlab 
links{1,2}.Inertia = [0.00455  0.00454 0.00029 0.00001 0.0 0.0];
links{1,3}.Inertia = [0.00032 0.0001 0.00042 0 0 0];
links{1,4}.Inertia = [0.00223 0.00219 0.00073 -0.00007 -0.00007 0];
links{1,5}.Inertia = [0.03844 0.01144 0.4988 +0.00111 +0.00112 0.00088];
links{1,6}.Inertia = [0.00277 0.00284 0.00012 0 0 0];
links{1,7}.Inertia = [0.0005 0.00281 0.00232 0 0 0];
links{1,8}.Inertia = [0.00795 0.01089 0.00294 +0.00029  +0.00029 0.00029];

% Calculating inertia matrices for coppeliaSim:
% Coppelia only takes Principal moments of Inertia which are eigenvalues of
% the Inertia tensor represented in the paper. By doing so for the 4th
% link, we can see that the eigenvalues are pretty close to diagonal elements of
% the inertia tensor. Also, the authors of the paper indicate that "the smallest diagonal
%term of the inertia of links one to five is forced to correspond to
%the axis parallel to the link length. The non-diagonal elements
%of the inertia matrix should be small compared to the diagonal ones"

% So, I took the 3 elements of Ixx Iyy Izz as PCI then changed the frame :

I_vrep(1,:) = diag (Rvrep12357*diag(I1(1:3))*Rvrep12357');
I_vrep(2,:) = diag (Rvrep12357*diag(I2(1:3))*Rvrep12357');
I_vrep(3,:) = diag (Rvrep12357*diag(I3(1:3))*Rvrep12357');
I_vrep(4,:) = diag (Rvrep4*diag(I4(1:3))*Rvrep4');
I_vrep(5,:) = diag (Rvrep12357*diag(I5(1:3))*Rvrep12357');
I_vrep(6,:) = diag (Rvrep6*diag(I6(1:3))*Rvrep6');
I_vrep(7,:) = diag (Rvrep12357*diag(I7(1:3))*Rvrep12357');

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
show(lbr14,'visuals','on')