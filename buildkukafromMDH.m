% Building  a rigidBodytree from MDH table in 'sturz 2017' paper, But this
% model is never used in any files in this project 
%mdh : [a alpha d theta]
mdhparams = [0 0 0 0;
            0 pi/2 0 0;
            0 -pi/2 0.42 0;
            0 -pi/2 0 0;
            0 pi/2 0.4 0;
            0 pi/2 0 0;
            0 -pi/2 0 0];
lbr14 = rigidBodyTree;

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');

setFixedTransform(jnt1,mdhparams(1,:),'mdh');
body1.Joint = jnt1;

addBody(lbr14,body1,'base');

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');
body7 = rigidBody('body7');
jnt7 = rigidBodyJoint('jnt7','revolute');

setFixedTransform(jnt2,mdhparams(2,:),'mdh');
setFixedTransform(jnt3,mdhparams(3,:),'mdh');
setFixedTransform(jnt4,mdhparams(4,:),'mdh');
setFixedTransform(jnt5,mdhparams(5,:),'mdh');
setFixedTransform(jnt6,mdhparams(6,:),'mdh');
setFixedTransform(jnt7,mdhparams(7,:),'mdh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;

addBody(lbr14,body2,'body1')
addBody(lbr14,body3,'body2')
addBody(lbr14,body4,'body3')
addBody(lbr14,body5,'body4')
addBody(lbr14,body6,'body5')
addBody(lbr14,body7,'body6')

show(lbr14);
