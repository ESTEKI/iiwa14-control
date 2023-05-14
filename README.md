This project is to test multiple non-linear control methods for position control of a robotic manipulator (2D and KUKA LBR iiwa R14).
 Controllers : PD ,Inverse dynamic, Robust Inv. Dyn.,Adaptive Inverse Dynamic, Passivity-based, Robust Passivity-based and Adaptive passivity-based controller

Refer to "Robot Modeling and Control", 2nd Edition by M. W. SPONG.

Main Files:
1) PositionCtrl_2DoF.m 
 A 2D planar robot is used to test different contol methods.

2) PositionCtrl_ode45.m
 In this code, Kuka LBR iiwa is used as the robot platform to be controlled in position mode. 
  
3) PositionCtrl_CoSim.m
 Here, matlab is connected to CoppelliaSim. In CopelliaSim, robot is controlled in joint torque mode and control input 'u' is calculated in matlab and will be send to simulation at each time step.