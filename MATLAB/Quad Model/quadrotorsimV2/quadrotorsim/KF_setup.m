% KF_setup.m           19/11/2013
%Quadrotor Sim 
% 
%
% Purpose: to declare initial values
%
%%
%% sensor noises
position_uncertainty_var = (20/3600*pi/180)^2*ones(3,1);
%% simulation set up
step_time = 0.5;                       % simulation step time(sec)
end_time  = 1000;                   % simulation end time (sec)
%end_time  = 86400;
%% attitude estimator gains
Tatd  = 0.5;                          % attitude estimator update time (sec)
Tqint = 0.5;                         % discrete quaternion integration period (sec)
Tsen_out = 0.5;                      % sensor output period (sec)
TkfProp = 0.5;                       % Kalman filter propagation period (sec)
KfupdatePeriodInCycle = 1;           % Kalman filter update period (propagation cycle)
f_bw_atd = 0.02;                     % attitude determination bandwidth (hz)
%f_bw_atd = 0.005;
zeta = 0.7;
Krp =  (2*pi*f_bw_atd)^2 * eye(3);
Kpp =  2*zeta*2*pi*f_bw_atd*eye(3);
qest0 = [0*1e-4; 0; 0; 1];                          % initial estimator quaternion
delta_west0 = zeros(3,1);                           % initial deviation of estimator angular rate (rad/sec)
max_delta_w = 0.1*pi/180;
delta_w_lim = 2e-4; %0.1/pi/Tqint;
delta_th_lim= 1e-4; %0.1*pi/180/Tqint;
q0 = [0; 0; 0; 1];   

%% for estimate error standard deviation prediction calculation
wn=sqrt(diag(Krp));
k=sqrt((wn.^4+4*zeta^2)./(4*zeta*wn));
%% for using Lyapunove equation to solve for expected estimation error
C=[1 0];  K=[Kpp(1,1);Krp(1,1)];      A=[0    1;0 0]-K*C; B=K; 
H=[1 0];  K=[Kpp(1,1);Krp(1,1)]*Tatd; F=[1 Tatd;0 1]-K*H; G=K;
%% Kalman filter setups
Fmat = [eye(3) TkfProp*eye(3);zeros(3,3) eye(3)];
Hmat = [eye(3)  zeros(3,3)];
therr0 = max([abs(qest0(1:3)); 5*1e-4]);  % initial error estimate, assuming q0=[0 0 0 1]
P0 = diag([therr0^2*ones(1,3) 3e-6^2*ones(1,3)]);
R = TkfProp*KfupdatePeriodInCycle*diag(position_uncertainty_var);%1e-3^2*eye(3)*
Q = diag([1e-5^2*ones(1,3), 1e-7^2*ones(1,3)])*TkfProp;
max_rate = pi/180;
P0 = diag([1e-32*ones(1,3) 1e-5^2*ones(1,3)]);
Q = diag([1e-5^2*ones(1,3), 5e-6^2*ones(1,3)])*TkfProp;
max_bias = 1*pi/180/3600;
%% start simulation
Tcapt = Tsen_out;                              % sim variable capture rate (sec)

