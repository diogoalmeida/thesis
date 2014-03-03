% Event rule derivation
clear all
syms qx qy qz qp qw c_phi c_theta theta_up theta_low phi_up phi_low


T=[c_phi*2*acos(qp)*qx/sqrt(1-qp*qp)-qp^3*qx*c_theta*(2*acos(qw))^2/2+qz*qp^3*c_theta*2*acos(qw)*qy/sqrt(1-qw^2);
   c_phi*2*acos(qp)*qy/sqrt(1-qp*qp)-qp^3*qx*c_theta*(2*acos(qw))^2/2+qz*qp^3*c_theta*2*acos(qw)*qy/sqrt(1-qw^2);
   qz*qp^4*c_theta*2*acos(qw)/sqrt(1-qw^2)];

%% for qp

syms B C E F gamma_theta 

%% 0<=phi <=phi_l
gamma_phi = 2*acos(qp);
T=[c_phi*gamma_phi*qx/sqrt(1-qp*qp)-qp^3*qx*C+qz*qp^3*E*gamma_theta*qy;
   c_phi*gamma_phi*qy/sqrt(1-qp*qp)-qp^3*qy*C-qz*qp^3*E*gamma_theta*qx;
   qz*qp^4*E*gamma_theta];

%% phi_l < phi <= phi_up

gamma_phi = phi_low
T=[c_phi*gamma_phi*qx/sqrt(1-qp*qp)-qp^3*qx*C+qz*qp^3*E*gamma_theta*qy;
   c_phi*gamma_phi*qy/sqrt(1-qp*qp)-qp^3*qy*C-qz*qp^3*E*gamma_theta*qx;
   qz*qp^4*E*gamma_theta];
%% phi_u < phi <= pi

gamma_phi = phi_low*(2*acos(qp)-pi)/(phi_up-pi);

T=[c_phi*gamma_phi*qx/sqrt(1-qp*qp)-qp^3*qx*C+qz*qp^3*E*gamma_theta*qy;
   c_phi*gamma_phi*qy/sqrt(1-qp*qp)-qp^3*qy*C-qz*qp^3*E*gamma_theta*qx;
   qz*qp^4*E*gamma_theta];


%% for qw

%% 0 <= theta <= theta_l

gamma_theta = 2*acos(qw);
int_theta = (2*acos(qw))^2/2;

T=[-qp^3*qx*c_theta*int_theta+qz*qp^3*c_theta*gamma_theta*qy/sqrt(1-qw^2);
   -qp^3*qy*c_theta*int_theta-qz*qp^3*c_theta*gamma_theta*qx/sqrt(1-qw^2);
   qz*qp^4*c_theta*gamma_theta/sqrt(1-qw^2)];

%% 0 <= theta <= theta_l

gamma_theta = theta_low;
int_theta = theta_low^2/2+theta_low*(2*acos(qw)-theta_low);

T=[-qp^3*qx*c_theta*int_theta+qz*qp^3*c_theta*gamma_theta*qy/sqrt(1-qw^2);
   -qp^3*qy*c_theta*int_theta-qz*qp^3*c_theta*gamma_theta*qx/sqrt(1-qw^2);
   qz*qp^4*c_theta*gamma_theta/sqrt(1-qw^2)];

%% 0 <= theta <= theta_l

gamma_theta = theta_low*(2*acos(qw)-pi)/(theta_up-pi);
int_theta = theta_low^2/2+theta_low*(theta_up-theta_low)+theta_low*((2*acos(qw))^2-theta_up^2)/(2*(theta_up-pi))+theta_low*pi*(theta_up-2*acos(qw))/(theta_up-pi);

T=[-qp^3*qx*c_theta*int_theta+qz*qp^3*c_theta*gamma_theta*qy/sqrt(1-qw^2);
   -qp^3*qy*c_theta*int_theta-qz*qp^3*c_theta*gamma_theta*qx/sqrt(1-qw^2);
   qz*qp^4*c_theta*gamma_theta/sqrt(1-qw^2)];