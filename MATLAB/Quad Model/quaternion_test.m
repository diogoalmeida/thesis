%% From roll-pitch-yaw to quaternion

clear all
roll=pi/3;
pitch=pi/5;
yaw=pi/8;

q=angle2quat(roll,pitch,yaw,'XYZ');
q=[q(2:4),q(1)];

q_z=[0 0 q(3) q(4)]./(sqrt(q(3)^2+q(4)^2));

A=[q_z(4) q_z(3)  0        0;
   -q_z(3) q_z(4) 0         0;
   0       0      q_z(4) q_z(3);
   0       0      -q_z(3) q_z(4);];

q_xy=A\q';
q_xy=q_xy';

q_linha=A*q_xy'

