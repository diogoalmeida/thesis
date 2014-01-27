function [Torques, phi] = Sat_Thrust_Control(Orientation, Desired_Orientation, Angular_Vel, last_phi)
% Computes the command torques that regulates the attitude of a quadrotor
% using the 'Fast and Saturating Thrust Direction Control' method



q = angle_to_quat(Orientation);
q_d = angle_to_quat(Desired_Orientation);

q_e = quat_mult(quat_conjugate(q),q_d);

q_e = q_e.*sign_l(q_e(4));

q_z = get_z_from_quat(q_e);

q_xy = quat_inv_multiply(q,q_z);


phi = 2*acos(q_xy(4))
w_x = Angular_Vel(1)
w_y = Angular_Vel(2)
w_z = Angular_Vel(3)

% Compute the torque field resulting from the potential energy

Torque_phi = 0;

if phi >= 0 && phi <= phi_low
    
    Torque_phi = c_phi * phi;
    
else if phi > phi_low && phi <= phi_up
        
        Torque_phi = c_phi * phi_low;
        
    else if phi > phi_up && phi <= pi
            
            Torque_phi = c_phi*phi_low * (pi-phi)/(pi-phi_up);
            
        end
    end
end


Torque_field = Torque_phi*(1/(sqrt(1-q_xy(4)^2)))*[q_xy(1) q_xy(2) 0]; % T(q)

% Compute the Damping Injection
Beta = 0;

if phi >= 0 && phi <=phi_up
    Beta = 1;
else if phi > phi_up && phi <=pi
        Beta = (pi-phi)/(pi-phi_up);
    end
end

d_xy_low = d_l; % Close to the equilibrium, we add positive damping on all the components of the angular acceleration (to prevent overshooting?)


% Farther away from the equilibrium, the damping strategy consists in
% supporting the acceleration while the system is far away from the
% equilibrium and switching to positive damping when entering the
% deceleration phase. The switching moment if given by a switch curve that
% I do not fully understand at the time.


switch_curve = -sqrt(abs(v_2^2-2*(brake_torque*(phi_low-phi))/J_x));

phi_dot=(phi-last_phi)*t_s;

% The acceleration-supporting damping will be positive if the derivative of
% phi is positive 

if phi_dot > v_1
    
    d_acc = -c_phi*phi_low/phi_dot + brake_torque/phi_dot;

else if phi_dot > 0 && phi_dot <=v_1
        
        d_acc = -c_phi*phi_low/v_1 + brake_torque/v_1; %% to account for a bounded damping
        
    else
        
        d_acc = 0; %% If the angular speed of phi is negative, the system is moving towards the desired orientation and no damping is required
        
    end
end

d_dec = -c_phi*phi_low/phi_dot - brake_torque/phi_dot;

d_mix = d_acc +(phi_dot-r*switch_curve)/((1-r)*switch_curve)*(d_dec-d_acc);

d_xy=0;
d_xy_up=0;

if phi_dot > r*switch_curve
    
    d_xy_up = d_acc;
    
else if r*switch_curve >= phi_dot && phi_dot > switch_curve
        
        d_xy_up = d_mix;
        
    else if switch_curve >= phi_dot
            
            d_xy_up = d_dec;
            
        end
        
    end
    
end

% Overal damping obtained by interpolation

if phi < phi_low
    
    d_xy=d_xy_low;
    
else if phi >= phi_low && phi < phi_low + delta_phi
        
        d_xy = d_xy_low+(phi-phi_low)/delta_phi*(d_xy_up-d_xy_low);
        
    else if phi >= phi_low + delta_phi
            
            d_xy = d_xy_up;
            
        end 
       
    end
    
end


% Definition of the Damping matrix

D_xy = Beta/(1-q_xy(4)^2)*(d_xy*[q_xy(1)^2 q_xy(1)*q_xy(2); q_xy(1)*q_xy(2) q_xy(2)^2] + d*[q_xy(2)^2 -q_xy(1)*q_xy(2); -q_xy(1)*q_xy(2) q_xy(1)^2]);

% k_1 is the minimum between 1 and the solving of a quadratic equation. If
% it is less than 1, the control torque in xy is saturated.

T_x = Torque_field(1);
T_y = Torque_field(2);

a = D_xy(1,1)*w_x+D_xy(1,2)*w_y;
b = D_xy(2,1)*w_x+D_xy(2,2)*w_y;

if (2*(T_x*a+T_y*b))^2-4*(a^2+b^2)*(T_x^2+T_y^2+torque_xy^2) >= 0
    k1_a = (2*(T_x*a+T_y*b) - sqrt((2*(T_x*a+T_y*b))^2-4*(a^2+b^2)*(T_x^2+T_y^2+torque_xy^2)))/(2*a^2+b^2);
    k1_b = (2*(T_x*a+T_y*b) + sqrt((2*(T_x*a+T_y*b))^2-4*(a^2+b^2)*(T_x^2+T_y^2+torque_xy^2)))/(2*a^2+b^2);
    
    if k1_a > 0
        k1 = k1_a;
    else if k1_b > 0
            k1 = k1_b;
        else
            k1 = 0;
        end
    end
else
    k1 = 1;
    
end

if k1 > 1
    k1 = 1;
end

k2 = torque_z/abs(d_z*w_z);

if k2 > 1
    k2 = 1;
end

D = [k1*D_xy zeros(2,1); zeros(1,2) k2*d_z];

Torques = Torque_field' - D*[w_x;w_y;w_z];