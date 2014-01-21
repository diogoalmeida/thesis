% Multiplies q * p = W_R(p)*q 3x1


function r = multipl_R(q,p)
    a=size(p);
    b=size(q);
    if a(1) ~= 4 || a(2) ~= 1
        disp('p must be a quaternion')
        return;
    elseif b(1) ~= 3 || b(2) ~= 1
        disp('q must be a 3x1-vector')
        return;
    else
        
        W_R= [p(4) p(3) -p(2);
            -p(3) p(4) p(1);
            p(2) -p(1) p(4);
            -p(1) -p(2) -p(3)];
        r = W_R*q;
        %}
    end
    
  
  