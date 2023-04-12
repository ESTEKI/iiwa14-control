% % S. Esteki:
% This program connects to 'No Control_sc1' Copelliasim model
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
    % work. maximum torques are located in iiwa-brochure.pdf
    lbrMaximumTorques =[320 320 176 176 110 40 40];
    for link = 1:7
      sim.simxSetJointMaxForce(clientID,lbrJointHandles(link),lbrMaximumTorques(link),sim.simx_opmode_blocking)
    end
    % for link = 1:7  % Just verification
    %     [rtn, force]=sim.simxGetJointMaxForce(clientID,lbrJointHandles(link),sim.simx_opmode_blocking)
    % end
    
    % Set all joint torques to 0 and its neccessary:
    for link = 1:7  
                [ returnCode2] = sim.simxSetJointForce(clientID,lbrJointHandles(link),0,sim.simx_opmode_oneshot)
    end
                sim.simxSynchronousTrigger(clientID);
        sim.simxSetJointTargetVelocity(clientID,lbrJointHandles(2),99999,sim.simx_opmode_oneshot);
       [ returnCode2]= sim.simxSetJointForce(clientID,lbrJointHandles(2),14,sim.simx_opmode_oneshot);
       %[rtc1,handleparent] =sim.simxGetObjectParent(clientID,lbrJointHandles(5),sim.simx_opmode_blocking)
        %[ returnCode2]= sim.simxSetJointForce(clientID,lbrJoint1,-10.0,sim.simx_opmode_blocking);
        % Now step a few times:
        for i=0:50
            %disp('Press a key to step the simulation!');
            pause(0.05);
            sim.simxSynchronousTrigger(clientID);
         %   [ returnCode2]= sim.simxSetJointForce(clientID,lbrJointHandles(7),20,sim.simx_opmode_oneshot);
        end
        
        % stop the simulation:
        sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);

        % Now close the connection to CoppeliaSim:    
        sim.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    sim.delete(); % call the destructor!
    
    disp('Program ended');
%end




