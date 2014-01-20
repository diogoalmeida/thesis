%defines q_b, q_d, q.

function q = set_quat(angles_b, angles_d)
q_b = angle_to_quat(angles_b(3),angles_b(2),angles_b(1));
q_b_conj = quat_conjugate(q_b);
q_d = angle_to_quat(angles_d(3),angles_d(2),angles_d(1));
q = quat_multiply(q_b_conj, q_d);
end