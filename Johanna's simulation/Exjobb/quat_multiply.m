% Multiplies q * p = W(p)*q 4x1


function r = quat_multiply(q,p)
    a=size(p);
    b=size(q);
    if a(1) ~= 4 || a(2) ~= 1
        disp('p must be a quaternion')
        return;
    elseif b(1) ~= 4 || b(2) ~= 1
        disp('q must be a quaternion')
        return;
    else
        W= [p(4) p(3) -p(2) p(1);
            -p(3) p(4) p(1) p(2);
            p(2) -p(1) p(4) p(3);
            -p(1) -p(2) -p(3) p(4)];
        r = W*q;
    end
    
  
  