%% PID value initializer

%% initial orientation

q = angle2quat(pi/4,pi/4,pi/4,'XYZ'); % roll,pitch,yaw

qo_i=q(1);
qv1_i=q(2);
qv2_i=q(3);
qv3_i=q(4);

%% Controller parameters

K_roll=0.8;
Ti_roll=0.000;
Td_roll=0.4;
K_pitch=0.8;
Ti_pitch=0.000;
Td_pitch=0.4;
K_yaw=0.8;
Ti_yaw=0.000;
Td_yaw=0.5;