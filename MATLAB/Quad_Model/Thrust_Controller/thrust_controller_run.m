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
global s_surf;
global phi_dotv;

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
J_z = 14*10^-3;
torque_xy = 0.15;
torque_z = 0.03;
m=0.835;
g=9.8;
km=342.4;
bm=5106.8;
d_l=0.16;

%%

T = 1.6; % Simulation time [s]

phi = zeros(floor(T/t_s),1);
phi_dotv = zeros(floor(T/t_s),1);
q = zeros(floor(T/t_s),4);
q_d = angle2quat(0,0,0,'XYZ');
q_d = [q_d(2:4) q_d(1)];
p = zeros(floor(T/t_s),3);
v = zeros(floor(T/t_s),3);
w = zeros(floor(T/t_s),3);
torques = zeros(floor(T/t_s),3);
roll = zeros(round(T/t_s),1);
pitch = zeros(round(T/t_s),1);
yaw = zeros(round(T/t_s),1);
s_surf = zeros(round(T/t_s),1);

thrust=m*g;
u = zeros(floor(T/t_s),4);

to_u = [km km km km;
        0 0 d*bm -d*bm ;
        d*bm -d*bm 0 0;
        -km -km km km];
    
% Initial conditions
phi_o = 170*pi/180;
q_o = angle_to_quat([0 0 0]);
w_o = [-2.5 0 0];

q_o = [sin(phi_o/2) 0 0 cos(phi_o/2)];

phi(:,1)=phi_o;

q(:,1)=q_o(1);
q(:,2)=q_o(2);
q(:,3)=q_o(3);
q(:,4)=q_o(4);

w(:,1)=w_o(1);
w(:,2)=w_o(2);
w(:,3)=w_o(3);



for t=3*t_s:t_s:T
    
    i=round(t/t_s);
    
    thrust_controller(q_d);
    
    
    u(i,:) = to_u\[thrust; torques(i,:)'];
    
    quadcopter(u(i,:));
    
   
    
end

 [roll pitch yaw] = quat2angle([q(:,4),q(:,1:3)],'XYZ');

%%

fontsize = 15;

% figure(1);
% 
% for i=1:3
%     
%     subplot(3,1,i);
%     hold on
%     title(i);
%     plot(t_s:t_s:T,p(:,i));
%     
% end

figure(2)

subplot(3,1,1);
hold on
title('roll');
plot(t_s:t_s:T,roll);
    
subplot(3,1,2);
hold on
title('pitch');
plot(t_s:t_s:T,pitch);

subplot(3,1,3);
hold on
title('yaw');
plot(t_s:t_s:T,yaw);

figure(3)
hold on
title('Control torques');
plot(t_s:t_s:T,torques(:,1),'r');
plot(t_s:t_s:T,torques(:,2),'g');
plot(t_s:t_s:T,torques(:,3));
legend({'$\tau_x$','$\tau_y$','$\tau_z$'},'interpreter', 'latex','fontsize',fontsize);

figure(4)
hold on
title('Switch curve and angle velocity');
plot(t_s:t_s:T,phi_dotv(:));
plot(t_s:t_s:T,s_surf(:),'k.');
legend({'$\dot \varphi$','$s(\varphi)$'},'interpreter', 'latex','fontsize',fontsize);

figure(5)
hold on
title('Angular velocities');
plot(t_s:t_s:T,w(:,1),'r');
plot(t_s:t_s:T,w(:,2),'g');
plot(t_s:t_s:T,w(:,3));
legend({'$\omega_x$','$\omega_y$','$\omega_z$'},'interpreter', 'latex','fontsize',fontsize);

figure(6)
hold on
title('Orientation');
plot(t_s:t_s:T,q(:,1),'r');
plot(t_s:t_s:T,q(:,2),'g');
plot(t_s:t_s:T,q(:,3),'b');
plot(t_s:t_s:T,q(:,4),'k');
legend({'$q_1$','$q_2$','$q_3$, $q_4$'},'interpreter', 'latex','fontsize',fontsize);


    



