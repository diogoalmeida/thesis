%% Quad init: initializes the necessary variables for the quadricopter
%% simulink model to work.
clear all

% Test values

J_x=7.8*10^-3;
J_y=7.8*10^-3;
J_z=10.22*10^-3;
m=0.835;
g=9.8;
km=342.4;
bm=5106.8;
d=0.16;

%% Step values for rotor inputs

u1 = 0.0001;
u2 = 0.00008;
u3 = 0.00009;
u4 = 0.00009;

%% Initial conditions

q = angle2quat(0,0,0,'XYZ');

qo_i=q(1);
qv1_i=q(2);
qv2_i=q(3);
qv3_i=q(4);