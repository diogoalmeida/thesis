function [ ] = thrust_controller( q_d )
%THRUST_CONTROLLER Implements the thrust direction controller
    
    global t t_s q torque_xy torque_z c_phi phi_low phi_up torques phi;
    
    
    i = round(t/t_s);
    
    if i > 1
    
        
        q_error = quat_mult(quat_conjugate(q(i-1,:)),q_d);

        q_hat = q_error.*sign_l(q_error(4));
        
        q_z = get_z_from_quat(q_hat);
        q_xy = quat_mult_inv(q_z,q_hat);
        
        
        q_x = q_xy(1);
        q_y = q_xy(2);
        qp = q_xy(4);
        
        phi(i) = 2*acos(qp);
        torque_phi = 0;
        
        if phi(i) >= 0 && phi(i) <= phi_low
            
            torque_phi = c_phi * phi(i); % small error -> torsion spring behavior
            
        else if phi(i) > phi_low && phi(i) <= phi_up
                
                torque_phi = c_phi*phi_low; % saturation -> big error
                
            else if phi(i) > phi_up && phi(i) <= pi
                    
                    torque_phi = c_phi*(pi-phi(i))/(pi-phi_up); % vanish to zero to avoid singularity
                    
                end
                
            end
            
        end
        
        if qp ~=1
            torque_field = torque_phi * (1/(sqrt(1-qp^2)))*[q_x, q_y 0];
        else
            torque_field = 0; %% qp==1 -> zero error
        end
        
        torques(i,:) = torque_field;
        
    end
    


end

