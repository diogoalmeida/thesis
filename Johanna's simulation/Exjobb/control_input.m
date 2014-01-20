%attitude controller
function [tau,phi_dot,theta_dot,s_phi,s_theta, theta] = control_input(q_xy,q_z,omega,J,tau_xy_max,tau_z_max)
% T(q)
    qx=q_xy(1); qy=q_xy(2); qp=q_xy(4); qz=q_z(3); qw=q_z(4);
    phi = 2*acos(qp); 
    theta = 2*acos(qw);
    
    a_phi = (1/sqrt(1 - qp^2)).*[qx qy 0]';
    a_ortogonal = (1/sqrt(1 - qp^2)).*[qy -qx 0]';
    a_z = [0 0 1]';
    

    % Parameters
    phi_l = 10 * pi/180; phi_u = 175 * pi/180;
    theta_l = 15 * pi/180; theta_u = 175 * pi/180;
    c_phi = 0.95*tau_xy_max/phi_l; c_theta = 0.95*tau_z_max/theta_l;
    
    % Calculating Tphi_phi
    LAMBDA_phi = calc_lambda(phi,phi_l,phi_u);
    Tphi_phi = c_phi * LAMBDA_phi .* a_phi;
    
    % Calculating Tphi_theta
    Integral_theta = calc_integral(theta, theta_l, theta_u, c_theta);
    Tphi_theta = -2 * (cos(phi/2))^3 * sin(phi/2) * Integral_theta .* a_phi; % without 2 in the paper
    
    % Calculating Tortogonal_theta
    LAMBDA_theta = calc_lambda(theta,theta_l,theta_u);
    Tortogonal_theta = (cos(phi/2))^4*c_theta*LAMBDA_theta*qz*sqrt(1-qp^2)/(qp*sqrt(1-qw^2)).*a_ortogonal; % In paper: Tortogonal_theta = (qz/sqrt(1-qw^2))*(cos(phi/2))^3*sin(phi/2)*c_theta*LAMBDA_theta.*a_ortogonal;
    
    % Calculating Tz_theta
    Tz_theta = (cos(phi/2))^4*c_theta*LAMBDA_theta*qz/(sqrt(1-qw^2)).*a_z;
    
    % Calculating T(q)
    T_q = Tphi_phi + Tphi_theta + Tortogonal_theta + Tz_theta;
% D(x)
    % Parameters
    delta_phi = 5*pi/180; del_phi = 1.1994*sqrt(4*J(1)*c_phi);%0.1999; 
    v_phimax = (0.95*tau_xy_max+c_phi*phi_l)/del_phi;%1.425; 
    r_phi = 0.75; v_phi = 0.1;
    T_phi = norm(Tphi_phi)-norm(Tphi_theta); phi_dot = -a_phi'*omega;
    s_phi = calc_s_angle(v_phimax, J(1), tau_xy_max, phi_l, phi);
    
    r_theta = 0.75; T_z = T_q(3); delta_theta = 5*pi/180; del_z = 1.23*sqrt(4*J(3,3)*c_theta);%0.0961;
    v_thetamax = (1.04*tau_z_max+c_theta*theta_l)/del_z;%0.624; 
    theta_dot_z = -(qz/sqrt(1-qw^2))*omega(3); theta_dot = -qz/sqrt(1-qw^2)*(sqrt(1-qp^2)/qp*a_ortogonal'+a_z')*omega;
    s_theta = calc_s_angle(v_thetamax, J(3,3), tau_z_max, theta_l, theta);

    d_phi = calc_d_phi(phi, phi_l, phi_u, delta_phi, del_phi, T_phi, phi_dot, tau_xy_max, v_phi, v_phimax, J, r_phi);
    d_ort = del_phi;
    D_xy = d_phi/(1-qp^2).*[qx^2 qx*qy;qx*qy qy^2] + d_ort/(1-qp^2).*[qy^2 -qx*qy;-qx*qy qx^2];
    % k_xy
    if norm(T_q(1:2)-D_xy*omega(1:2))>tau_xy_max
        a = (D_xy(1,1)*omega(1)+D_xy(1,2)*omega(2))^2+(D_xy(2,1)*omega(1)+D_xy(2,2)*omega(2))^2;
        b = T_q(1)*(D_xy(1,1)*omega(1)+D_xy(1,2)*omega(2))+T_q(2)*(D_xy(2,1)*omega(1)+D_xy(2,2)*omega(2));
        d = T_q(1)^2 + T_q(2)^2 - tau_xy_max^2;
        k1 = b/a + sqrt((b/a)^2 - d/a);
        k2 = b/a - sqrt((b/a)^2 - d/a);
        if (b/a)^2 - d/a < 0    %avoid imaginary roots OBS THEY EXIST -> TORQUE NOT BOUNDED
            k_xy = 1;
            disp('imag roots')
        else
            if k1 > 0 && k2 >0
                k_xy = min([1 k1 k2]);
           %     disp('k1, k2 >0')
            elseif k1 > 0
                k_xy = min(1,k1);
            %    disp('k1 >0')
            elseif k2 > 0
                k_xy = min(1,k2);
            %   disp('k2 >0')
            else
                k_xy = 1;
            %  disp('k1,k2<0')
            end
        end
    else
        k_xy = 1;
    end

    %k_z TO BE DEFINED AFTER d_z
    d_z = calc_chi(theta, theta_l, delta_theta, del_z, T_z, tau_z_max, theta_dot_z, theta_dot, r_theta,v_thetamax,J(3,3),theta_u);
    if T_q(3) - d_z*omega(3) > tau_z_max
        k_z = (T_q(3)-tau_z_max)/(d_z*omega(3));
        k_z = min(1,k_z);
    elseif T_q(3) - d_z*omega(3) < -tau_z_max
        k_z = (T_q(3)+tau_z_max)/(d_z*omega(3));
        k_z = min(1,k_z);
    else
        k_z = 1;
    end
       D_x = [k_xy*D_xy [0 0]';          % 3x3 matrix >=0 
               0 0 k_z*d_z];
    
    %{
    T_q
    D_x
    omega
    phi_dot
    phi
    s_phi
    %}
    tau = T_q - D_x*omega;
end


function lambda = calc_lambda(angle, angle_l, angle_u)
    if angle <= angle_l
        lambda = angle;
    elseif angle > angle_l && angle < angle_u
        lambda = angle_l;
    elseif angle >= angle_u && angle <= pi
        lambda = angle_l * (angle - pi)/(angle_u - pi);
    else
        disp('Error in function calc_lambda: check value of qp/qw')
        return;
    end
end  


function integral = calc_integral(angle, angle_l, angle_u, c_angle)
    if angle <= angle_l
        integral = 0.5 * c_angle * angle^2;
    elseif angle > angle_l && angle <= angle_u
        E_l = 0.5 * c_angle * angle_l^2;
        integral = E_l + c_angle * angle_l * (angle - angle_l);
    elseif angle > angle_u && angle <= pi
        E_l = 0.5 * c_angle * angle_l^2;
        E_m = c_angle * angle_l * (angle_u - angle_l);
        integral = E_l + E_m + c_angle*angle_l / (pi-angle_u) * (pi*(angle-angle_u) - 0.5*(angle^2-angle_u^2));
    else
        disp('Error in function calc_integral: check value of qp/qw')
        return;
    end    
end

% Damping D_xy

function d_phi = calc_d_phi(phi, phi_l, phi_u, delta_phi, del_phi, T_phi, phi_dot, tau_xy_max, v_phi, v_phimax, J, r_phi)
    if phi < 0
        disp('Error in function calc_d_phi: angle <0')
    elseif phi <= phi_l
        d_phi = del_phi;
    elseif phi > phi_l && phi <= (phi_l + delta_phi)
        d_phi = (phi_l+delta_phi-phi)/delta_phi*del_phi + (phi-phi_l)/delta_phi*calc_del_phi_star(phi_dot, calc_s_angle(v_phimax, J(1), tau_xy_max, phi_l, phi), r_phi, T_phi, tau_xy_max, v_phi);%calc_del_phi_star to be defined
    elseif phi > (phi_l+delta_phi) && phi <= pi
        d_phi = calc_chi_d_phi(phi_u, delta_phi, del_phi, T_phi, phi_dot, tau_xy_max, v_phi, v_phimax, J, phi_l, phi,r_phi); 
    end
        %disp('calc_d_phi')

end

function chi_d_phi = calc_chi_d_phi(phi_u, delta_phi, del_phi, T_phi, phi_dot, tau_xy_max, v_phi, v_phimax, J, phi_l, phi,r_phi)
    if phi < 0
        disp('Error in function calc_d_phi: angle <0')
    elseif phi <= phi_u-delta_phi
        chi_d_phi = calc_del_phi_star(phi_dot, calc_s_angle(v_phimax, J(1), tau_xy_max, phi_l, phi), r_phi, T_phi, tau_xy_max, v_phi); 
    elseif phi > phi_u-delta_phi && phi <= phi_u
        chi_d_phi = (phi_u-phi)/delta_phi*calc_del_phi_star(phi_dot, calc_s_angle(v_phimax, J(1), tau_xy_max, phi_l, phi), r_phi, T_phi, tau_xy_max, v_phi) + (phi-phi_u+delta_phi)/delta_phi*del_phi;
    elseif phi > phi_u && phi <= pi
        chi_d_phi = del_phi;
    end
        %disp('calc_chi_d_phi')

end

function del_phi_star = calc_del_phi_star(phi_dot, s_phi, r_phi, T_phi, tau_xy_max, v_phi)
    if phi_dot <= s_phi
        del_phi_star = calc_del_phi_star_dec(T_phi, phi_dot, tau_xy_max); 
    elseif phi_dot > s_phi && phi_dot <= r_phi*s_phi 
        del_phi_star = (s_phi*r_phi-phi_dot)/(s_phi*(r_phi-1))*calc_del_phi_star_dec(T_phi, phi_dot, tau_xy_max) + (phi_dot-s_phi)/(s_phi*(r_phi-1))*calc_del_phi_star_acc(T_phi, phi_dot, tau_xy_max, v_phi);%calc_del_phi_star_dec/acc to be defined
    elseif phi_dot > r_phi*s_phi
        del_phi_star = calc_del_phi_star_acc(T_phi, phi_dot, tau_xy_max, v_phi);    
    end
        %disp('calc_del_phi_star')

end

function s_angle = calc_s_angle(v_anglemax, J_angle, tau_angle_max, angle_l, angle)
    s_angle = -sqrt(v_anglemax^2-2/J_angle*tau_angle_max*(angle_l-angle));
        %disp('calc_s_angle')

end

function del_phi_star_dec = calc_del_phi_star_dec(T_phi, phi_dot, tau_xy_max)
    del_phi_star_dec = -T_phi/phi_dot - tau_xy_max/phi_dot;
        %disp('calc_del_phi_star_dec')

end

function del_phi_star_acc = calc_del_phi_star_acc(T_phi, phi_dot, tau_xy_max, v_phi)
    if phi_dot > v_phi
        del_phi_star_acc = -T_phi/phi_dot + tau_xy_max/phi_dot;
    elseif phi_dot > 0 && phi_dot <= v_phi
        del_phi_star_acc = -T_phi/v_phi + tau_xy_max/v_phi;
    elseif phi_dot <= 0
        del_phi_star_acc = 0;
    end
        %disp('calc_del_phi_star_acc')

end

% Damping D_z

function chi = calc_chi(theta, theta_l, delta_theta, del_z, T_z, tau_z_max, theta_dot_z, theta_dot, r_theta,v_thetamax,J_2,theta_u)
    if theta <= theta_l
       chi = del_z; 
    elseif theta > theta_l && theta < theta_l + delta_theta
       chi = (theta_l+delta_theta-theta)/(delta_theta)*del_z +(theta - theta_l)/(delta_theta)*calc_chi2(theta,theta_u,delta_theta,del_z, theta_l, tau_z_max, theta_dot, r_theta,v_thetamax,J_2,T_z, theta_dot_z);
    elseif theta > theta_l + delta_theta
       chi = calc_chi2(theta,theta_u,delta_theta,del_z, theta_l, tau_z_max, theta_dot, r_theta,v_thetamax,J_2,T_z, theta_dot_z);
    end
        %disp('calc_chi')

end

function chi2 = calc_chi2(theta,theta_u,delta_theta,del_z, theta_l, tau_z_max, theta_dot, r_theta,v_thetamax,J_2,T_z, theta_dot_z)
    if theta <= theta_u-delta_theta
        chi2 = calc_d_z_star(theta, theta_l, tau_z_max, theta_dot, r_theta,v_thetamax,J_2,T_z, theta_dot_z);
    elseif theta > theta_u-delta_theta && theta <= theta_u
        chi2 = (theta_u-theta)/(delta_theta)*calc_d_z_star(theta, theta_l, tau_z_max, theta_dot, r_theta,v_thetamax,J_2,T_z, theta_dot_z) + (theta-theta_u+delta_theta)/(delta_theta)*del_z;
    elseif theta > theta_u
        chi2 = del_z;
    end
        disp('calc_chi2')

end

function d_z_star = calc_d_z_star(theta, theta_l, tau_z_max, theta_dot, r_theta,v_thetamax,J_2,T_z, theta_dot_z)
%v_thetamax, J_2, tau_z_max, theta_l, theta
    s_theta = calc_s_angle(v_thetamax, J_2, tau_z_max, theta_l, theta);
    if theta_dot <= s_theta
        d_z_star = calc_d_z_star_dec(T_z, tau_z_max, theta_dot_z);
    elseif theta_dot > s_theta && theta_dot <= r_theta*s_theta
        d_z_star = (r_theta*s_theta - theta_dot)/(s_theta*(r_theta-1))*calc_d_z_star_dec(T_z, tau_z_max, theta_dot_z) + (theta_dot-s_theta)/(s_theta*(r_theta-1))*calc_d_z_star_acc(T_z, tau_z_max, theta_dot_z);
    elseif theta_dot > r_theta*s_theta
        d_z_star = calc_d_z_star_acc(T_z, tau_z_max, theta_dot_z);
    end
     %disp('calc_d_z_star')

end

function d_z_star_acc = calc_d_z_star_acc(T_z, tau_z_max, theta_dot_z)
    %disp('calc_d_z_star_acc')
    
    v_theta = 0.1;
    if theta_dot_z > v_theta
        d_z_star_acc = -abs(T_z)/theta_dot_z + tau_z_max/theta_dot_z;
    elseif theta_dot_z > 0 && theta_dot_z <= v_theta
        d_z_star_acc = -abs(T_z)/v_theta + tau_z_max/v_theta;
    elseif theta_dot_z <= 0
        d_z_star_acc = 0;
    end
end

function d_z_star_dec = calc_d_z_star_dec(T_z, tau_z_max, theta_dot_z)
    %disp('calc_d_z_star_dec')
    v_theta = 0.1;
    if theta_dot_z < -v_theta
        d_z_star_dec = -abs(T_z)/theta_dot_z - tau_z_max/theta_dot_z;
    elseif theta_dot_z < 0 && theta_dot_z >= -v_theta
        d_z_star_dec = -abs(T_z)/v_theta - tau_z_max/v_theta;
    elseif theta_dot_z >= 0
        d_z_star_dec = 0;
    end
end