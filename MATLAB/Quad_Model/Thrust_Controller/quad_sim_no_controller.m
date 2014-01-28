%% Global variables used for the simulation

clear all
close all

format shortG

global t_s;
global t;
global phi_low;
global phi_up;
global delta_phi;
global c_phi;
global d;
global d_z;
global brake_torque;
global v_1;
global v_2;
global r;
global J_x;
global J_y;
global J_z;
global torque_xy;
global torque_z;
global m;
global g;
global km;
global bm;
global d_l;
global phi;
global q;
global torques;
global w;
global p;
global v;

t_s = 10^-3;
t = 0;
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

%%

T = 10; % Simulation time [s]

phi = zeros(floor(T/t_s),1);
q = zeros(floor(T/t_s),4);
q(:,4)=1;
p = zeros(floor(T/t_s),3);
v = zeros(floor(T/t_s),3);
w = zeros(floor(T/t_s),3);
torques = zeros(floor(T/t_s),3);

u1 = 0.0001;
u2 = 0.00008;
u3 = 0.00009;
u4 = 0.00009;

u=ones(floor(T/t_s),4);
u(:,1)=u1;
u(:,2)=u2;
u(:,3)=u3;
u(:,4)=u4;

for t=t_s:t_s:T
    
    quadcopter(u(round(t/t_s),:));
    
end

for i=1:3
    
    figure(i)
    plot(t_s:t_s:T,p(:,i));
    
end



