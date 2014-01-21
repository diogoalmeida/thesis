%% Quad init: initializes the necessary variables for the quadricopter
%% simulink model to work.


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

u1=0.0002;
u2=0.0002;
u3=0.00021;
u4=0.00019;
