

%% Simulation of the Event-Triggered control of a linear plant

clear all
close all

global t t_s x

t = 0;
t_s = 10^-3;
T = 10;

x = zeros(round(T/t_s),2);
error = zeros(round(T/t_s),2);
u = zeros(round(T/t_s),1);

execution_period = 10^-1;

x(1,:)=[10,10];
x(2,:)=[10,10];

K = [1 -4];
u = 0;

u_pulse = zeros(round(T/t_s),1);

execution_counter = execution_period/t_s-1;
% 
% % Fixed period
% for t=2*t_s:t_s:T
%     
%     i = round(t/t_s);
%     
%     execution_counter=execution_counter+1;
%     if execution_counter == execution_period/t_s
%         u(i) = K*x(i-1,:)';
%         u_pulse(i)=1;
%         execution_counter = 0;
%     else
%         u(i) = u(i-1);
%     end
%     
%     plant(u(i));
% 
% end

x_fixed = x;
x = zeros(round(T/t_s),2);
u = zeros(round(T/t_s),1);

sig = 0.04;

x(1,:)=[10,10];
x(2,:)=[10,10];

last_x = x(1,:);

hit=0;

for t = 2*t_s:t_s:T
   
    i = round(t/t_s);
    
    
    if norm(error(i-1,:)) >= sig * norm(x(i-1,:))
        
        hit = hit + 1;
        u(i) = K*x(i-1,:)';
        u_pulse(i)=1;
        plant(u(i));
        last_x = x(i,:);
    
    else
        u(i) = u(i-1);
        plant(u(i));
    end
    
   
    
    error(i,:) = last_x - x(i,:);
    
    
    
end

%% 

hit

figure(1);
hold on
plot(t_s:t_s:T,x(:,1));
plot(t_s:t_s:T,x(:,2),'r');

figure(2)
hold on
plot(t_s:t_s:T,u);
stem(t_s:t_s:T,u_pulse*max(u)/2);