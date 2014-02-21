%% Global variables used for the simulation

clear all
close all

format shortG
 global t t_s p v w_true s_surf_phi v_dot eps_error ticks old_x alpha sigma s_surf_theta q theta_dotv torque_xy torque_z d c_phi phi_low phi_up theta_low theta_up torques phi v_1_phi v_1_theta v_2_theta v_2_phi small_delta_phi small_delta_theta r_phi r_theta delta_phi w s_surf_phi phi_dotv J_x J_y J_z m g km bm q_error c_theta theta delta_theta;


% Controller parameters
phi_low = 10*pi/180;
theta_low = 15*pi/180;
phi_up = 175*pi/180;
theta_up = 175*pi/180;
delta_phi = 5*pi/180;
delta_theta = 5*pi/180;
small_delta_phi = 0.1999;
small_delta_theta = 0.0961; 
c_phi = 0.817;
c_theta = 0.109;
v_1_phi = 0.1;
v_2_phi = 1.425;
v_1_theta = 0.1;
v_2_theta = 0.624;
r_phi = 0.75;
r_theta = 0.75;


% Simulation parameters (plant and sampling)
t_s = 10^-3;
t = 0;
m=0.835;
g=9.8;
km=342.4;
bm=5106.8;
d = 0.16;
J_x = 8.5*10^-3;
J_y = 8.5*10^-3;
J_z = 14*10^-3;
torque_xy = 0.15;
torque_z = 0.03;

%%

T = 5; % Simulation time [s]

phi = zeros(floor(T/t_s),1);
phi_dotv = zeros(floor(T/t_s),1);
theta = zeros(floor(T/t_s),1);
theta_dotv = zeros(floor(T/t_s),1);
q = zeros(floor(T/t_s),4);
q_error = zeros(floor(T/t_s),4);
ticks = zeros(floor(T/t_s),1);
v_dot = zeros(floor(T/t_s),1); 
q_d = angle2quat(0,0,0,'XYZ');
q_d = [q_d(2:4) q_d(1)];
p = zeros(floor(T/t_s),3);
v = zeros(floor(T/t_s),3);
w = zeros(floor(T/t_s),3);
w_true = zeros(floor(T/t_s),3);
torques = zeros(floor(T/t_s),3);
roll = zeros(round(T/t_s),1);
pitch = zeros(round(T/t_s),1);
yaw = zeros(round(T/t_s),1);
s_surf_phi = zeros(round(T/t_s),1);
s_surf_theta = zeros(round(T/t_s),1);
alpha = 0;
sigma = 0.9;
eps_error = 0.1;

thrust=m*g;
u = zeros(floor(T/t_s),4);

to_u = [km km km km;
        0 0 d*bm -d*bm ;
        d*bm -d*bm 0 0;
        km km -km -km];
    
% Initial conditions
phi_o = 170*pi/180;
theta_o = 10*pi/180;
q_o = angle_to_quat([0 0 0]);
w_o = [0 0 1.7];

q_o = [-0.992 -0.087 0.008 0.087];
old_x = zeros(1,7); % to ensure that there is an initial control update


phi(:,1)=phi_o;


q(:,1)=q_o(1);
q(:,2)=q_o(2);
q(:,3)=q_o(3);
q(:,4)=q_o(4);

w(:,1)=w_o(1);
w(:,2)=w_o(2);
w(:,3)=w_o(3);

w_true=w;


for t=3*t_s:t_s:T
    
    i=round(t/t_s);
    
    event_attitude_controller2(q_d);
    
    
    u(i,:) = to_u\[thrust; torques(i,:)'];
    
    quadcopter(u(i,:));
    
  % insert noise
   
   %w(i,:) = w(i,:) + 0.1*rand(1,3);
    
end

 [roll pitch yaw] = quat2angle([q(:,4),q(:,1:3)],'XYZ');

%%

fontsize = 15;
line = 1;

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

% figure(2)
% 
% subplot(3,1,1);
% hold on
% title('roll');
% plot(t_s:t_s:T,roll);
%     
% subplot(3,1,2);
% hold on
% title('pitch');
% plot(t_s:t_s:T,pitch);
% 
% subplot(3,1,3);
% hold on
% title('yaw');
% plot(t_s:t_s:T,yaw);


% figure(3)
% hold on
% title('Orientation');
% plot(t_s:t_s:T,q_error(:,1),'r');
% plot(t_s:t_s:T,q_error(:,2),'g');
% plot(t_s:t_s:T,q_error(:,3),'b');
% plot(t_s:t_s:T,q_error(:,4),'k');
% legend({'$q_1$','$q_2$','$q_3$', '$q_4$'},'interpreter', 'latex','fontsize',fontsize);

figure(4)
hold on
title('Angular velocities');
plot(t_s:t_s:T,w(:,1),'r');
plot(t_s:t_s:T,w(:,2),'g');
plot(t_s:t_s:T,w(:,3));
legend({'$\omega_x$','$\omega_y$','$\omega_z$'},'interpreter', 'latex','fontsize',fontsize);


figure(1);
subplot(3,1,1);
hold on
title('Control torques');
plot(t_s:t_s:T,torques(:,1),'r','Linewidth',line);
plot(t_s:t_s:T,torques(:,2),'g','Linewidth',line);
plot(t_s:t_s:T,torques(:,3),'--k','Linewidth',line);
plot(t_s:t_s:T,sqrt(torques(:,1).^2+torques(:,2).^2),'--c','Linewidth',line);
legend({'$\tau_x$','$\tau_y$','$\tau_z$','$||\tau_{xy}||$'},'interpreter', 'latex','fontsize',fontsize);

% figure(6)
% hold on
% title('Switch curve and angle velocity');
% plot(t_s:t_s:T,phi_dotv(:));
% plot(t_s:t_s:T,s_surf_phi(:),'k.');
% plot(t_s:t_s:T,theta_dotv(:),'g');
% plot(t_s:t_s:T,s_surf_theta(:),'r.');
% legend({'$\dot \varphi$','$s(\varphi)$'},'interpreter', 'latex','fontsize',fontsize);


%figure(70)
subplot(3,1,2);
hold on
title('$\varphi$ and $\vartheta$','interpreter','latex');
plot(t_s:t_s:T,phi,'Linewidth',line);
plot(t_s:t_s:T,theta,'r','Linewidth',line);

subplot(3,1,3);
hold on
title('Sampling instants');
stem(t_s:t_s:T,ticks,'Marker','none');
axis([0 T 0 2]);
    
figure(8)
subplot(2,1,1);
hold on
title('$\dot V(x)$','interpreter','latex');
plot(t_s:t_s:T,v_dot,'Linewidth',line);
subplot(2,1,2);
hold on
title('Sampling instants');
stem(t_s:t_s:T,ticks,'Marker','none');
axis([0 T 0 2]);

disp('Ticks:');
disp(sprintf(strcat(sprintf('%d',sum(ticks)),' out of ',sprintf(' %d',size(ticks,1)))));

disp('tau x energy');
disp(sum(torques(:,1).^2)/T);

disp('tau y energy');
disp(sum(torques(:,2).^2)/T);

disp('tau z energy');
disp(sum(torques(:,3).^2)/T);

