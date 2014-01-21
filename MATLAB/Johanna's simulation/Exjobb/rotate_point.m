%rotates a point p with a quaternion q, rotate_point(p,q)

function p_rot = rotate_point(p,q)
    p_rotate = quat_multiply(quat_multiply(q,point),quat_conjugate(q));
end


