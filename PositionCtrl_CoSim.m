% % S. Esteki:
% This program connects to 'LBRiiwa14_TorqueCtrlMode.ttt' Copelliasim model
% 
% using the remote API synchronous mode. The synchronous mode needs to be
% pre-enabled on the server side. You would do this by
% starting the server (e.g. in a child script) with:

% % S. Esteki: Add a child script in e.g. 'box' object in the vrep scene
% and add this command :
% simRemoteApi.start(19999,1300,false,true) % PORT NUMBER MUST REMAIN 19999 not
% 19997! no its wrong...
% 
% But we try to connect on port 19997
% where there should be a continuous remote API
% server service already running and pre-enabled for
% synchronous mode. (it is enabled)
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

%function positionCtrl1_NoC()
    disp('Program started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

dt = 0.005;
tf = 2; %final time


    if (clientID>-1)
        disp('Connected to remote API server');

        % enable the synchronous mode on the client:
        sim.simxSynchronous(clientID,true);

        % start the simulation:
        sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);
        sim.simxSynchronousTrigger(clientID);%step 1
        pause(0.1);
        [returnCode,lbrJointHandles] = sim.simxGetObjects(clientID,1,sim.simx_opmode_blocking); % get all (lbr) joint handles. '1' means sim_object_joint_type
                                                                                                
    % [ returnCode,lbrJoint1]=sim.simxGetObjectHandle( clientID,'./LBRiiwa14R820/joint1',sim.simx_opmode_blocking);
    % Torque limits must be set by the program unless setting target force wont
    % work. maximum torques are located in iiwa-brochure.pdf
    lbrMaximumTorques =[320 320 176 176 110 40 40];

    sim.simxSetJointPosition(clientID,lbrJointHandles(4),0,sim.simx_opmode_blocking);
    lbrJointPositions_Prev = zeros(7,1);
    lbrJointPosition = lbrJointPositions_Prev;
    lbrJointVelocity = zeros(7,1);
    lbrJointVelocity_prev = lbrJointVelocity;
    lbrJointAccel = zeros(7,1);
    q= zeros(7,50);
    dq =q;
    ddq = q;
    uu= q;%comanded torques

    for link = 1:7
          [rtn,lbrJointPositions_Prev(link)] = sim.simxGetJointPosition(clientID,lbrJointHandles(link),sim.simx_opmode_oneshot);% there was a bug so I had to read positions here (all joints returned 43!)
    end
       
    %step 2
    sim.simxSynchronousTrigger(clientID);      

    for link = 1:7
          [rtn,lbrJointPositions_Prev(link)] = sim.simxGetJointPosition(clientID,lbrJointHandles(link),sim.simx_opmode_oneshot);%do not use blocking modes in main loop, so time-consuming
    end
    

    for link = 1:7
      sim.simxSetJointMaxForce(clientID,lbrJointHandles(link),lbrMaximumTorques(link),sim.simx_opmode_blocking);
    end
    % for link = 1:7  % Just for verification
    %     [rtn, force]=sim.simxGetJointMaxForce(clientID,lbrJointHandles(link),sim.simx_opmode_blocking)
    % end
    
    % Set all joint torques to 0 and its neccessary:
    for link = 1:7  
         sim.simxSetJointForce(clientID,lbrJointHandles(link),0,sim.simx_opmode_oneshot);
    end
                  
      % Trajectory 
      %qdesired = [0; 20*pi/180.0; 0; 45*pi/180.0; 0 ;0 ;0];
      qdesired = [0.0 ;20.0; 0 ;-110.0; 0 ;-40.0; 90.0;]*pi/180;

      [qTr, dqTr, ddqTr, ppTr] = jointSpaceTrajectory(lbrJointPositions_Prev,qdesired,0,2);
lbrJointPositions_Prev*180/pi
        try
        % MAIN LOOP:
        NumStep = tf/dt;
        for step=3:NumStep
            sim.simxSynchronousTrigger(clientID);
            for link = 1:7
              [rtn,lbrJointPosition(link)] = sim.simxGetJointPosition(clientID,lbrJointHandles(link),sim.simx_opmode_oneshot);%do not use blocking modes in main loop, so time-consuming
            end
            timeIndex = uint16(ceil(step*dt*100))+1;
                qd = [qTr(1,timeIndex); qTr(2,timeIndex); qTr(3,timeIndex); qTr(4,timeIndex); qTr(5,timeIndex) ;qTr(6,timeIndex) ;qTr(7,timeIndex)];
                dqd = [dqTr(1,timeIndex); dqTr(2,timeIndex); dqTr(3,timeIndex); dqTr(4,timeIndex); dqTr(5,timeIndex) ;dqTr(6,timeIndex) ;dqTr(7,timeIndex)];
                ddqd =  [ddqTr(1,timeIndex); ddqTr(2,timeIndex); ddqTr(3,timeIndex); ddqTr(4,timeIndex); ddqTr(5,timeIndex) ;ddqTr(6,timeIndex) ;ddqTr(7,timeIndex)];

          lbrJointVelocity = (lbrJointPosition-lbrJointPositions_Prev)/dt;
          lbrJointAccel = (lbrJointVelocity-lbrJointVelocity_prev)/dt;
            KP = eye(7,7)*20;
            KD = eye(7,7) *10;
            %qd = [0; 20*pi/180.0; 0; 45*pi/180.0; 0 ;0 ;0];
            %qdesired = [0.0 ;20.0; 0 ;-110.0; 0 ;-40.0; 90.0;]*pi/180;
            qtilda = qd - lbrJointPosition;
            dqtilda = dqd - lbrJointVelocity;
            %PD controller
            %u = KP*qtilda - KD*lbrJointVelocity + lbr14.gravityTorque(lbrJointPosition);
            
            aq =0+ KP*qtilda + KD*dqtilda ;
            u = lbr14.massMatrix(lbrJointPosition)*aq+ lbr14.velocityProduct(lbrJointPosition,lbrJointVelocity) + lbr14.gravityTorque(lbrJointPosition); %velocityProduct = C(q,dq)*dq

          q(:,step) = lbrJointPosition;
          dq(:,step) = lbrJointVelocity;
          ddq(:,step) = lbrJointAccel;
          lbrJointPositions_Prev = lbrJointPosition;
          lbrJointVelocity_prev = lbrJointVelocity;
            for link = 1:7
                if u(link)<0
                    sim.simxSetJointTargetVelocity(clientID,lbrJointHandles(link),-99999,sim.simx_opmode_oneshot); 
                else
                    sim.simxSetJointTargetVelocity(clientID,lbrJointHandles(link),99999,sim.simx_opmode_oneshot); 
                end
                 if abs(u(link)) > lbrMaximumTorques(link)
                    u(link) = lbrMaximumTorques(link);
                 end
                  sim.simxSetJointForce(clientID,lbrJointHandles(link),abs(u(link)),sim.simx_opmode_oneshot);
                 
            end
          %pause(dt);
                uu(:,step)=u;
        end
        %END MAIN LOOP
        catch ME
            cprintf('Error','Error in main loop: %s \r\n', ME.message);%better not to use cprintf anymore!
            pause(0.1);
        end
            pause(10);

        % stop the simulation:
        sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);

        % Now close the connection to CoppeliaSim:    
        sim.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    sim.delete(); % call the destructor!
    
close all;

hold on 
for l = 1:7
plot(q(l,:));
end
title('Joint Position');
legend

figure;
hold on
plot(tvec, qTr);
title('Desired Trajectory');
legend

figure;
hold on
for l = 1:7
plot(dq(l,:));
end
title('Joint Velocity');




figure;
hold on
for l = 1:7
plot(ddq(l,:));
end
title('Joint Acceleration');

figure;
hold on
for l = 1:7
plot(uu(l,:));
end
title('Joint Torque');
legend;
disp('Program ended');
%end




