close all
clear all

% define orientation N->B
roll_b = rand;%5*pi/180;
pitch_b = -rand;%0;
yaw_b = rand/4;
angles_b = [roll_b pitch_b yaw_b];
%define orientation N->D
roll_d = 0;
pitch_d = 5*pi/180;
yaw_d = 0;
angles_d = [roll_d pitch_d yaw_d];

q = set_quat(angles_b,angles_d);
%q = [0.992 0.087 -0.008 0.087]'; 
norm(q);
q_hat = q./sign_hat(q);
J=10^(-3).*diag([19.6 19.6 37.6]);
%J=10^(-3).*diag([8.5 8.5 14]);
%omega = [0 0 1.7]';
omega=[rand rand rand/10]';     %initial value

tau_xy_max = 0.2;
%tau_z_max = 0.1;
tau_z_max = tau_xy_max/5; %default: 0.03
dt=.01;
t_end = 2.6;
X=zeros(7,t_end/0.1+1);
X(:,1) = [q' omega'];
ANGLES=zeros(2,t_end/0.1);
ANGLES_DOT=zeros(2,t_end/0.1);
S_ANGLE=zeros(2,t_end/0.1);
TAU = zeros(3,t_end/0.1);
%T_PHI = zeros(1,t_end/0.1);
i = 1;
for t= 0:dt:t_end
    %t
    tic;
    % calculate q_z
    qz = q_hat(3)/sqrt(q_hat(3)^2+q_hat(4)^2);
    qw = q_hat(4)/sqrt(q_hat(3)^2+q_hat(4)^2);
    q_z = [0 0 qz qw]';
    norm_q_z = norm(q_z);
    % calculate q_xy
    Qz_trans =[qw -qz 0 0;
               qz qw 0 0;
               0 0 qw -qz;
               0 0 qz qw];
    q_xy = Qz_trans*q_hat;
    norm(q_xy);
    phi = 2*acos(q_xy(4)); %theta = 2*acos(qw);
    [tau,phi_dot,theta_dot,s_phi,s_theta, theta] = control_input(q_xy,q_z,omega,J,tau_xy_max,tau_z_max);
    %tau = [tau_xy_max;0;0];
    %[tau, T_phi] = control_input2(q_xy,omega,J,phi);
    domega = J\(cross(J*omega,omega)) + J\tau;
    dq = -0.5.*multipl_R(omega,q);
    %update q and omega    
    q = q + dq.*dt;
    q = q./norm(q);
    q_hat = q./sign_hat(q);
    norm_q = norm(q);
    omega = omega + domega.*dt;
    i = i+1;
    X(:,i) = [q' omega']; 
    ANGLES(:,i-1) = [phi; theta];
    ANGLES_DOT(:,i-1) = [phi_dot; theta_dot];
    S_ANGLE(:,i-1) = [s_phi; s_theta];
    TAU(:, i-1) = tau;
    %T_PHI(i-1)=T_phi;
    time = toc;
end
t=(0:dt:t_end+dt);

    figure(1)
    plot(t,X(1,:),'b'),hold on
    plot(t,X(2,:),'g')
    plot(t,X(3,:),'r')
    plot(t,X(4,:),'k'),hold off, title('q')
%}
    figure(2)
    plot(t,X(5,:),'b'),hold on
    plot(t,X(6,:),'r')
    plot(t,X(7,:),'g'),hold off, title('omega: wx = blue, wy=r, wz=green')

    %}


    phi_l = 10 * pi/180; phi_u = 175 * pi/180; delta_phi = 5*pi/180;
    figure(3)
    plot((0:dt:t_end),ANGLES(1,:),'r'),hold on
    plot((0:dt:t_end),ANGLES(2,:),'g'),hold on
    plot((0:dt:t_end),phi_l,'k-')
    plot((0:dt:t_end),phi_l+delta_phi,'k-')
    plot((0:dt:t_end),phi_u,'k-')
    plot((0:dt:t_end),phi_u-delta_phi,'k-'),title('angles'),hold off

%}

%Ploting tau, k1,k2 ok: tau_xy < tau_xy_max     tau_x o tau_y gör hopp i
%0.36 (pga att phi_dot korsar kurvan rphi*sphi) och 0.64 (pga att phi_dot korsar kurvan s_phi)

figure(4)
plot((0:dt:t_end),TAU(1,:),'b'),hold on
plot((0:dt:t_end),tau_xy_max,'k-')
%figure(11)
plot((0:dt:t_end),TAU(2,:),'r'),
plot((0:dt:t_end),-tau_xy_max,'k-')
%figure(12)
norm_tau_xy = norm_tau(TAU(1:2,:));
plot((0:dt:t_end),norm_tau_xy,'c')
plot((0:dt:t_end),-tau_z_max,'k-')
%figure(13)
plot((0:dt:t_end),TAU(3,:),'g'),title('blue = tau_x, red = tau_y, green = tau_z')
plot((0:dt:t_end),tau_z_max,'k-'),hold off
%}
%{
% T_phi ok: T_phi < t_xy_max always
figure(14)          
tau_xy_max = 0.15;
tau_z_max = 0.03;
plot((0:dt:t_end),T_PHI,'b'),hold on
plot((0:dt:t_end),tau_xy_max,'k'),hold on
plot((0:dt:t_end),-tau_xy_max,'k'),hold off
%}
t1=(0:dt:t_end);

figure(5)
plot(t1,ANGLES_DOT(1,:),'r'), hold on
plot(t1,S_ANGLE(1,:),'k')
plot(t1,ANGLES_DOT(2,:),'g')
plot(t1,S_ANGLE(2,:),'k')
%plot(t1,0.75*S_ANGLE(2,:),'g-'),
hold off,title('red=phi_{dot}, black = s_{phi}, green=theta_{dot}, black = s_{theta}')

%}
