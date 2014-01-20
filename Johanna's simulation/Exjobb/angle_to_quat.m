%From yaw,pitch,roll to quaternion

function q = angle_to_quat(psi,theta,phi)


qx=[sin(phi/2) 0 0 cos(phi/2)]';
qy=[0 sin(theta/2)  0 cos(theta/2)]';
qz=[0 0 sin(psi/2) cos(psi/2)]';
b=quat_multiply(qz,qy); 
q = quat_multiply(b,qx); 
end
