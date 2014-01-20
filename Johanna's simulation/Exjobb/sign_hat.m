function sign = sign_hat(q)
    if length(q) ~= 4
        disp('Error in sign_hat: q must be 4x1')
    else
        if q(4) >= 0
            sign = 1;
        elseif q(4) < 0
            sign = -1;
        end        
    end

end