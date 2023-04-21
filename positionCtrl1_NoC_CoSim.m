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

dt = 0.05;
numOfIterations = 50;

    if (clientID>-1)
        disp('Connected to remote API server');

        % enable the synchronous mode on the client:
        sim.simxSynchronous(clientID,true);

        % start the simulation:
        sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);
        sim.simxSynchronousTrigger(clientID);
        
        [returnCode,lbrJointHandles] = sim.simxGetObjects(clientID,1,sim.simx_opmode_blocking); % get all (lbr) joint handles. '1' means sim_object_joint_type
                                                                                                
    % [ returnCode,lbrJoint1]=sim.simxGetObjectHandle( clientID,'./LBRiiwa14R820/joint1',sim.simx_opmode_blocking);
    % Torque limits must be set by the program unless setting target force wont
    % work. maximum torques are from iiwa-brochure.pdf
    lbrMaximumTorques =[320 320 176 176 110 40 40];
    lbrJointPositions_Prev = zeros(7,1);
    lbrJointPosition = lbrJointPositions_Prev;
    lbrJointVelocity = zeros(7,1);
    lbrJointVelocity_prev = lbrJointVelocity;
    lbrJointAccel = zeros(7,1);
    q= zeros(7,numOfIterations);
    dq =q;
    ddq = q;

    for link = 1:7
      sim.simxSetJointMaxForce(clientID,lbrJointHandles(link),lbrMaximumTorques(link),sim.simx_opmode_blocking);
     % [rtn,lbrJointPosition(link)] = sim.simxGetJointPosition(clientID,lbrJointHandles(link),sim.simx_opmode_blocking)
    end
    % for link = 1:7  % Just verification
    %     [rtn, force]=sim.simxGetJointMaxForce(clientID,lbrJointHandles(link),sim.simx_opmode_blocking)
    % end
    
    % Set all joint torques to 0 and its neccessary:
    for link = 1:7  
         sim.simxSetJointForce(clientID,lbrJointHandles(link),0,sim.simx_opmode_oneshot);
         [rtn,lbrJointPosition(link)] = sim.simxGetJointPosition(clientID,lbrJointHandles(link),sim.simx_opmode_oneshot); %there was a bug so I had to read positions here (all joints returned 43!)
    end
                  
        %sim.simxSynchronousTrigger(clientID);
        %sim.simxSetJointTargetVelocity(clientID,lbrJointHandles(4),-99999,sim.simx_opmode_oneshot);
        %[returnCode2]= sim.simxSetJointForce(clientID,lbrJointHandles(4),1,sim.simx_opmode_oneshot);
        try
        % MAIN LOOP:
        for i=1:50
            sim.simxSynchronousTrigger(clientID);
            for link = 1:7
              [rtn,lbrJointPosition(link)] = sim.simxGetJointPosition(clientID,lbrJointHandles(link),sim.simx_opmode_oneshot);%do not use blocking modes in main loop, so time-consuming
            end
          lbrJointVelocity = (lbrJointPosition-lbrJointPositions_Prev)/dt;
          lbrJointAccel = (lbrJointVelocity-lbrJointVelocity_prev)/dt;
          q(:,i) = lbrJointPosition;
          dq(:,i) = lbrJointVelocity;
          ddq(:,i) = lbrJointAccel;
          pause(dt);
          lbrJointPositions_Prev = lbrJointPosition;
          lbrJointVelocity_prev = lbrJointVelocity;
         %   [ returnCode2]= sim.simxSetJointForce(clientID,lbrJointHandles(7),20,sim.simx_opmode_oneshot);
        end
        %END MAIN LOOP
        catch ME
            cprintf('Error','Error in main loop: %s \r\n', ME.message);%better not to use cprintf anymore!
            pause(0.1);
        end

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

figure;
hold on
for l = 1:7
plot(dq(l,:));
end

figure;
hold on
for l = 1:7
plot(ddq(l,:));
end


    disp('Program ended');
%end




