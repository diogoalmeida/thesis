function [ theta_out ] = correct_theta( theta_in )
% Returns theta between -pi to pi.

theta_out=rem(theta_in,2*pi);
if theta_out>pi
    theta_out=theta_out-2*pi;
elseif theta_out<-pi
    theta_out=theta_out+2*pi;
end