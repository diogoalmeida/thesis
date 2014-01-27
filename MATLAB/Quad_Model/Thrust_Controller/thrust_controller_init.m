%% variables used for the simulation

t_s = 10^-5;

phi_low = 10*pi/180;
phi_up = 175*pi/180;
delta_phi = 5*pi/180;
c_phi = 0.817;
d = 0.1999;
d_z = 0.1666;
brake_torque = 0.15;
v_1 = 0.1;
v_2 = 1.425;
r = 0.75;

J_x = 8.5*10^-3;
J_y = 8.5*10^-3;
J_z = 8.5*10^-3;

torque_xy = 0.15;
torque_z = 0.03;

m=0.835;
g=9.8;
km=342.4;
bm=5106.8;
d_l=0.16;

q = angle2quat(pi/3,pi/3,0,'XYZ');

qo_i=q(1);
qv1_i=q(2);
qv2_i=q(3);
qv3_i=q(4);
