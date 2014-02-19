function [ ] = quadcopter( u )
%QUADCOPTER Simulates the quadcopter behavior for inputs u=[u1; u2; u3; u4]
    
    global t_s t p v q w w_true km g m d bm J_x J_y J_z torques;
    
    
    
    if t > 2*t_s
        
        i=round(t/t_s);
        
        
        
        q_loc = q(i-1,:)';
        
        % from quaternion to rotation matrix that maps the inertial frame
        % into the body frame
        R = [q_loc(1)^2-q_loc(2)^2-q_loc(3)^2+q_loc(4)^2, 2*(q_loc(1)*q_loc(2)+q_loc(4)*q_loc(3)), 2*(q_loc(1)*q_loc(3)-q_loc(4)*q_loc(2));
             2*(q_loc(2)*q_loc(1)-q_loc(4)*q_loc(3)), -q_loc(1)^2+q_loc(2)^2-q_loc(3)^2+q_loc(4)^2, 2*(q_loc(2)*q_loc(3)+q_loc(4)*q_loc(1));
             2*(q_loc(3)*q_loc(1)+q_loc(4)*q_loc(2)), 2*(q_loc(3)*q_loc(2)-q_loc(4)*q_loc(1)), -q_loc(1)^2-q_loc(2)^2+q_loc(3)^2+q_loc(4)^2];
         
        
        
        % the sum of motor inputs times a constant gives the overall thrust
        T = km*sum(u(:));   
        
        % the quad speed in the inertial frame has a gravity component plus
        % the thrust aligned with the inertial frame
        v(i,:) = v(i-1,:)+(g*[0;0;1]-1/m*R'*[0;0;T])'*t_s;
        p(i,:) = p(i-1,:)+v(i,:).*t_s;
 
        
        tau_loc = zeros(3,1);
        % the torques are given by the differences between individual motor
        % inputs
        
        tau_loc(1) = d*bm*(u(3)-u(4)); % around x
        tau_loc(2) = d*bm*(u(1)-u(2)); % around y
        tau_loc(3) = km*(-u(3)-u(4)+u(1)+u(2)); % around z
                
        
        J=[J_x 0 0;
           0 J_y 0;
           0  0 J_z];
      
        
        w_loc = w_true(i-1,:)';
        
        % Cross product tensor
        
        w_cross = [0, -w_loc(3) w_loc(2);
                   w_loc(3), 0, -w_loc(1);
                   -w_loc(2), w_loc(1), 0];
        
        
        % The angular speed is given by Euler's equation
        
        w_true(i,:) = w_loc' + (J\(J*w_cross*w_loc) + J\tau_loc)'*t_s;
        w(i,:) = w_true(i,:);
        
%         if i==2
%             disp('initial conditions check:');
%             disp('initial angular speed around x:');
%             disp(w(i-1,1));
%             disp('resulting angular speed with current torque:');
%             disp(w(i,1));
%             pause();
%         end
        
        
        % the evolution of the attitude is given by the attitude kinematics
        % [Survey of Attitude Representations]
        
        E = [q_loc(4), -q_loc(3), q_loc(2); 
            q_loc(3), q_loc(4), -q_loc(1);
            -q_loc(2), q_loc(1), q_loc(4);
            -q_loc(1), -q_loc(2), -q_loc(3)];
        
        
        q(i,:) = (q_loc + (0.5*E*w_true(i,:)')*t_s)';
        
%         % From the paper
%         
%          W_r=[q_loc(4) q_loc(3) -q_loc(2);
%                -q_loc(3) q_loc(4) q_loc(1);
%                q_loc(2) -q_loc(1) q_loc(4);
%                -q_loc(1) -q_loc(2) -q_loc(3)];
%         
%         q(i,:) = (q_loc - (0.5*W_r*w(i,:)')*t_s)';
       
        q(i,:) = q(i,:)./(norm(q(i,:)));
    end
        
        

end

