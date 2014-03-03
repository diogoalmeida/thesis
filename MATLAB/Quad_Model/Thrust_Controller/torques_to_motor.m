 syms d c_T c_D u1 u2 u3 u4 t1 t2 t3 T
 
 N = [-d*c_T d*c_T 0 0;
      0      0     d*c_T -d*c_T;
      -c_D  -c_D  c_D   c_D;
      c_T   c_T   c_T  c_T]
  
  inv(N)