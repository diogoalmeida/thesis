% implementation of thrust direction control

function [tau, T_phi] = control_input2(q_xy,omega,J,phi)
% q_xy and q_z, phi and theta
    qx=q_xy(1); qy=q_xy(2); qp=q_xy(4);  
    a_xy = (1/sqrt(1 - qp^2)).*[qx qy 0]';
    %norm(a_xy)
    
    
    % Parameters of phi
    c_phi = 0.817; phi_l = 10 * pi/180; phi_u = 175 * pi/180; delta_phi = 5*pi/180;
    
    % Calculating T_phi
    T_phi = calc_T_phi(phi, c_phi, phi_l, phi_u);
    
    % Calculating T(q)
    T_q = T_phi*a_xy;
    
    
% D(x)
    % Parameters
    d_z = 0.1666; d= 0.1999; r= 0.75; v_1 = 0.1; v_2 = 1.425; tau_br = 0.15; tau_xy_max = 0.15; tau_z_max = 0.03;
    
    % D_xy
    d_xy = calc_d_xy(phi,phi_l,delta_phi,d, a_xy,omega,r,v_2, J(1), tau_br, c_phi, v_1);
    beta_x = calc_beta_x(phi,phi_u);
    D_xy = beta_x/(1-qp^2).*(d_xy.*[qx^2 qx*qy;qx*qy qy^2] + d.*[qy^2 -qx*qy;-qx*qy qx^2]);
    
    % k_1
    if norm(T_q(1:2)-D_xy*omega(1:2))>tau_xy_max
        a = (D_xy(1,1)*omega(1)+D_xy(1,2)*omega(2))^2+(D_xy(2,1)*omega(1)+D_xy(2,2)*omega(2))^2;
        b = T_q(1)*(D_xy(1,1)*omega(1)+D_xy(1,2)*omega(2))+T_q(2)*(D_xy(2,1)*omega(1)+D_xy(2,2)*omega(2));
        d = T_q(1)^2 + T_q(2)^2 - tau_xy_max^2;
        k1 = b/a + sqrt((b/a)^2 - d/a);
        k2 = b/a - sqrt((b/a)^2 - d/a);
        if (b/a)^2 - d/a < 0    %avoid imaginary roots OBS THEY EXIST -> TORQUE NOT BOUNDED
            k_1 = 1;
            disp('imag roots')
        else
            if k1 > 0 && k2 >0
                k_1 = min([1 k1 k2]);
           %     disp('k1, k2 >0')
            elseif k1 > 0
                k_1 = min(1,k1);
            %    disp('k1 >0')
            elseif k2 > 0
                k_1 = min(1,k2);
            %   disp('k2 >0')
            else
                k_1 = 1;
            %  disp('k1,k2<0')
            end
        end
    else
        k_1 = 1;
    end

    % k_2 
    k_2 = tau_z_max/abs(d_z*omega(3));
    k_2 = min(1,k_2);
    
    % D_x
    D_x = [k_1*D_xy [0 0]';          % 3x3 matrix >=0 
           0 0 k_2*d_z];
    % tau
    tau = T_q - D_x*omega;
end

%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function T_phi = calc_T_phi(phi, c_phi, phi_l, phi_u)
    if phi <= phi_l && phi >= 0
        T_phi = c_phi*phi;
    elseif phi > phi_l && phi <= phi_u
        T_phi = c_phi*phi_l;
    elseif phi > phi_u && phi <= pi
        T_phi = c_phi*phi_l * (pi - phi)/(pi - phi_u);
    else
        disp('Error in function calc_lambda: check value of qp/qw')
        return;
    end
end  

function beta_x = calc_beta_x(phi,phi_u)
    if phi >= 0 && phi <= phi_u
        beta_x = 1;
    elseif phi > phi_u && phi <= pi
        beta_x = (pi - phi)/(pi - phi_u);
    end
end

function d_xy = calc_d_xy(phi,phi_l,delta_phi,d, a_xy,omega,r,v_2, J_1, tau_br, c_phi, v_1)
    if phi < phi_l
        d_xy = d;
    elseif phi >= phi_l && phi < (phi_l+delta_phi)
        d_xy = calc_d_xymix(d, phi, phi_l, delta_phi, a_xy,omega,r,v_2, J_1, tau_br, c_phi, v_1); 
    elseif phi >= (phi_l+delta_phi)
        d_xy = calc_d_xyII(a_xy,omega,r,v_2, J_1, tau_br, phi_l, phi,c_phi, v_1);
    end
end

function d_xyII = calc_d_xyII(a_xy,omega,r,v_2, J_1, tau_br, phi_l, phi,c_phi, v_1)
    phi_dot = -a_xy'*omega;
    s_phi = calc_s_phi(v_2, J_1, tau_br, phi_l, phi);
    if phi_dot > r*s_phi
        d_xyII = calc_d_acc(c_phi, phi_l, tau_br, phi_dot, v_1);
    elseif phi_dot <= r*s_phi && phi_dot > s_phi
        d_xyII = calc_d_mix(phi_dot, r, s_phi, c_phi, phi_l, tau_br, v_1);
    elseif phi_dot <= s_phi
        d_xyII = calc_d_dec(c_phi, phi_l, phi_dot, tau_br);
    end
end

function s_phi = calc_s_phi(v_2, J_1, tau_br, phi_l, phi)
    s_phi = -sqrt(v_2^2-2/J_1*tau_br*(phi_l-phi));
end

function d_acc = calc_d_acc(c_phi, phi_l, tau_br, phi_dot, v_1)
    if phi_dot > v_1
        d_acc = -c_phi*phi_l/phi_dot + tau_br/phi_dot; 
    elseif phi_dot > 0 && phi_dot <= v_1
        d_acc = -c_phi*phi_l/v_1 + tau_br/v_1;
    elseif phi_dot <= 0
        d_acc = 0;
    end
end

function d_dec = calc_d_dec(c_phi, phi_l, phi_dot, tau_br)
    d_dec = -c_phi*phi_l/phi_dot - tau_br/phi_dot;
end

function d_mix = calc_d_mix(phi_dot, r, s_phi, c_phi, phi_l, tau_br, v_1)
    d_acc = calc_d_acc(c_phi, phi_l, tau_br, phi_dot, v_1);
    d_dec = calc_d_dec(c_phi, phi_l, phi_dot, tau_br);
    d_mix = d_acc + (phi_dot - r*s_phi)/((1-r)*s_phi)*(d_dec - d_acc);
end

function d_xymix = calc_d_xymix(d, phi, phi_l, delta_phi, a_xy,omega,r,v_2, J_1, tau_br, c_phi, v_1)
    d_xyII = calc_d_xyII(a_xy,omega,r,v_2, J_1, tau_br, phi_l, phi,c_phi, v_1);
    d_xymix = d + (phi - phi_l)/delta_phi*(d_xyII - d);
end