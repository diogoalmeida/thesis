function [ ] = plant( u )
%PLANT Simulates the linear plant from Paulo Tabuada's example
    
    global x t t_s;

    A = [0 1;
        -2 3];
    
    B = [0;1];
    
    i = round(t/t_s);
   
    
    x(i,:) = (x(i-1,:)'+(A*x(i-1,:)' + B*u)*t_s)';

end

