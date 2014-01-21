function q_star = quat_conjugate(q)
    q_prim = -1.*q;
    q_star = [q_prim(1:3); q(4)];
end