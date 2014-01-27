function [ q_o ] = quat_inv_multiply(q,q_inv )
%QUAT_INV_MULTIPLY Returns the matrix inverse of W(q_inv) times q

     W=[q_inv(4) q_inv(3) -q_inv(2) q_inv(1);
       -q_inv(3) q_inv(4) q_inv(1) q_inv(2);
       q_inv(2) -q_inv(1) q_inv(4) q_inv(3);
       -q_inv(1) -q_inv(2) -q_inv(3) q_inv(4)];
   
   q_o=W\q';
   
   q_o=q_o';


end

