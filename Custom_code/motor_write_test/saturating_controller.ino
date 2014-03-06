#include <quaternion_diogo.h>
#include <saturating_controller.h>


/*
* Lambda function: Saturation that vanishes close to PI
*/
float lambda_f(float up,float low,float val)
{

	if(val >= 0 && val <= low)
		return val;
	else if(val>low && val <= up)
		return low;
	else if(val>up && val <= PI)
		return low*(val-PI)/(up-PI);
	else
		return 0; // safeguard?

}

/*
* Integral lambda function: Computes the integral of the lambda function
*/
float integral_lambda_f(float up, float low, float val)
{
	if(val >= 0 && val <= low)
		return 0.5f*val*val;
	else if(val > low && val <= up)
		return 0.5f*low*low+low*(val-low);
	else if(val > up && val <= PI)
		return 0.5f*low*low+low*(up-low)+0.5f*low*(val*val-up*up)/(up-PI)-PI*low*(val-up)/(up-PI);
	else
		return 0;
}
/*
* Xi function: Linear interpolation between f1 and f2
*/
float xi_f(float up, float low, float f1, float f2, float val)
{
	if(val <= low)
		return f1;
	else if(val > low && val <= up)
		return (up-val)/(up-low)*f1 + (val-low)/(up-low)*f2;
	else
		return f2;
}

/*
* Double Xi function: Linear interpolation from f1 to f2 and back to f1
*/
float double_xi_f(float up1, float up2, float low1, float low2,float f1, float f2,float val)
{
	return xi_f(low2,low1,f1,xi_f(up2,up1,f2,f1,val),val);
}

/*
* Safe Arccos: Computs the arc cosine of a value, dealing with eventual rounding errors on x
*/

float safe_acos (float x)
  {
	  if (x < -1.0) 
	  	x = -1.0 ;
	  else if (x > 1.0) 
	  	x = 1.0 ;

	  return acos (x) ;
  }

/*
* Compute Phi: Returns the displacement angle of the thrust direction
*/
float compute_phi(float qp)
{
	return 2*safe_acos(qp);
}

/*
* Compute Theta: Returns the displacement angle of the yaw angle
*/
float compute_theta(float qw)
{
	return 2*safe_acos(qw);
}

/*
*	Compute artificial torques: computes the artificial torques vector for the control signal
*/
Vector3<float> compute_art_torques(Quaternion_D qxy, Quaternion_D qz,float phi, float theta)
{
	float qx=qxy.q1,qy=qxy.q2,qzz=qz.q3,qp=qxy.q4,qw=qz.q4;
	float A=0,B=0,T_1=0,T_2=0,T_3=0;

	if(qp!=1)
		A=c_phi*lambda_f(phi_up,phi_low,phi)/safe_sqrt(1-qp*qp)-qp*qp*qp*c_theta*integral_lambda_f(theta_up,theta_low,theta);
	else
		A=0;


	if(qw!=1)
		B=qzz*qp*qp*qp*c_theta*lambda_f(theta_up,theta_low,theta)/safe_sqrt(1-qw*qw);
	else
		B=0;
	

	T_1 = qx*A+B*qy;
	T_2 = qy*A-B*qx;
	T_3 = B*qp;

	Vector3<float> T(T_1,T_2,T_3);

	return T;
}

/*
*	Compute Phi dot: Computes the time derivative of the displacement angle of the thurst axis
*/
float compute_phi_dot(Quaternion_D qxy,Vector3<float> omega_f)
{
	float qx=0,qy=0,qp=0,wx=0,wy=0;

	qx = qxy.q1;
	qy = qxy.q2;
	qp = qxy.q4;
	wx = omega_f.x;
	wy = omega_f.y;

	if(qp!=1)
		return -(qx*wx+qy*wy)/safe_sqrt(1-qp*qp);
	else
		return 0;


}

/*
*	Compute Theta dot: Computes the time derivative of the yaw error angle
*/
float compute_theta_dot(Quaternion_D qxy, Quaternion_D qz, Vector3<float> omega_f)
{
	float qx=0,qy=0,qzz=0,qp=0,qw=0,wx=0,wy=0,wz=0,A=0;

	qx = qxy.q1;
	qy = qxy.q2;
	qzz = qz.q3;
	qp = qxy.q4;
	qw = qz.q4;
	wx = omega_f.x;
	wy = omega_f.y;
	wz = omega_f.z;

	if(qw!=1)
		A = qzz/safe_sqrt(1-qw*qw);
	else
		A=0;

	if(qp!=1)
		return -A*qy*wx/qp + A*qx*wy/qp - A*wz;

	return -A*wz;

}

float compute_theta_dot_hole(Quaternion_D qz, Vector3<float> omega_f)
{
	float qzz=0,qw=0,wz=0;

	qzz = qz.q3;
	qw = qz.q4;
	wz = omega_f.z;

	if(qw!=1)
		return -qzz*wz/safe_sqrt(1-qw*qw);

	return 0;

}


/*
*	Compute Switch curve Phi: Computes the switch curve between accelerating and decelerating damping on the thrust axis movement
*/
float compute_switch_curve_phi(float phi)
{
	return -safe_sqrt(v_phi_max*v_phi_max-2*torque_xy_max*(phi_low-phi)/J_x);
}

/*
*	Compute Switch curve Theta: Computes the switch curve between accelerating and decelerating damping on the yaw
*/
float compute_switch_curve_theta(float theta)
{
	return -safe_sqrt(v_theta_max*v_theta_max-2*torque_z_max*(theta_low-theta)/J_z);
}

/*
* Computes the 'available torque' in the phi direction (discounting the torque used by theta movement)
*/
float compute_torque_phi(Quaternion_D qxy,float phi, float theta,Vector3<float> omega_f)
{
	float qx=0,qy=0,qp=0,A=0,B=0;

	qx = qxy.q1;
	qy = qxy.q2;
	qp = qxy.q4;

	if (qp!=1)
		A = c_phi * lambda_f(phi_up,phi_low,phi)/safe_sqrt(1-qp*qp);
	else
		A = 0;

	B = -c_theta * qp*qp*qp *integral_lambda_f(theta_up,theta_low,theta);

	return safe_sqrt(A*qx*A*qx+A*qy*A*qy)-safe_sqrt(B*qx*B*qx+B*qy*B*qy);
}

/*
* Computes the torque in the yaw direction
*/
float compute_torque_z(Quaternion_D qxy, Quaternion_D qz,float theta,Vector3<float> omega_f)
{
	float qzz=0, qw=0,qp=0;

	qp = qxy.q4;
	qzz = qz.q3;
	qw = qz.q4;

	if (qw!=1)
		return safe_sqrt(sq(qzz*qp*qp*qp*qp*c_theta*lambda_f(theta_up,theta_low,theta)/safe_sqrt(1-qw*qw)));

	return 0;

}

/*
* Computes the accelerating damping on the thrust axis
*/
float compute_acc_damping_phi(float phi_dot, float T_phi)
{
	if(phi_dot > v_phi)
		return (-T_phi+torque_xy_max)/phi_dot;
	
	if(phi_dot > 0 && phi_dot <= v_phi)
		return (-T_phi+torque_xy_max)/v_phi;

	return 0;
}

/*
* Computes the decelerating damping on the thrust axis
*/
float compute_dec_damping_phi(float phi_dot, float T_phi)
{
	return -(T_phi+torque_xy_max)/phi_dot;
}

/*
* Computes the star damping gain on the thrust axis
*/
float compute_star_damping_phi(float phi_dot,float dec_phi, float acc_phi, float switch_phi)
{
	return xi_f(r_phi*switch_phi,switch_phi,dec_phi,acc_phi,phi_dot);
}

/*
* Computes the final damping gain on the thrust axis
*/
float compute_damping_phi(Quaternion_D qxy,Quaternion_D qz, Vector3<float> omega_f,float phi,float theta)
{
	float phi_dot=0,switch_phi=0,T_phi=0, phi_star = 0,phi_dec = 0,phi_acc = 0;

	phi_dot = compute_phi_dot(qxy,omega_f);
	switch_phi = compute_switch_curve_phi(phi);
	T_phi = compute_torque_phi(qxy,phi,theta,omega_f);
	phi_dec = compute_dec_damping_phi(phi_dot,T_phi);
	phi_acc = compute_acc_damping_phi(phi_dot,T_phi);
	phi_star = compute_star_damping_phi(phi_dot,phi_dec,phi_acc,switch_phi);

	//hal.console->printf_P(PSTR("phi:[%.7f,%.7f,%.7f,%.7f,%.7f]\r\n"),phi_dec,phi_star,phi_acc,phi_dot,T_phi);

	return double_xi_f(phi_up-delta_phi,phi_up,phi_low,phi_low+delta_phi,small_delta_phi,phi_star,phi);
}

float compute_acc_damping_z(float theta_dot_hole, float T_z)
{
	if(theta_dot_hole > v_theta)
		return (-T_z+torque_z_max)/theta_dot_hole;
	
	if(theta_dot_hole > 0 && theta_dot_hole <= v_theta)
		return (-T_z+torque_z_max)/v_theta;

	return 0;
}

float compute_dec_damping_z(float theta_dot_hole, float T_z)
{
	if(theta_dot_hole < -v_theta)
		return (-T_z-torque_z_max)/theta_dot_hole;
	
	if(theta_dot_hole < 0 && theta_dot_hole >= -v_theta)
		return (-T_z-torque_z_max)/v_theta;

	return 0;
}
float compute_star_damping_z(float theta_dot,float theta_dot_hole,float dec_theta, float acc_theta, float switch_theta)
{
	return xi_f(r_theta*switch_theta,switch_theta,dec_theta,acc_theta,theta_dot);
}
float compute_damping_z(Quaternion_D qxy,Quaternion_D qz, Vector3<float> omega_f,float phi,float theta)
{
	float theta_dot=0,theta_dot_hole=0,switch_theta=0,T_z=0, d_star_z=0 ,d_dec_z=0, d_acc_z=0, d_xi=0;

	theta_dot = compute_theta_dot(qxy,qz,omega_f);
	theta_dot_hole = compute_theta_dot_hole(qz,omega_f);
	switch_theta = compute_switch_curve_theta(theta);
	T_z = compute_torque_z(qxy, qz,theta, omega_f);

	d_dec_z = compute_dec_damping_z(theta_dot_hole, T_z);
	d_acc_z = compute_acc_damping_z(theta_dot_hole, T_z);
	d_star_z = compute_star_damping_z(theta_dot,theta_dot_hole,d_dec_z,d_acc_z,switch_theta);
	d_xi = double_xi_f(theta_up-delta_theta,theta_up,theta_low,theta_low+delta_theta,small_delta_z,d_star_z,theta);



	return xi_f(phi_up,phi_up-delta_phi,d_xi,small_delta_z,phi);
}

/*
*	Computes the gain that allows to saturate the xy torque
*/
float compute_kxy(Vector3<float> art_torques,Vector3<float> omega_f, Quaternion_D qxy,float d_phi)
{
	float qx=0, qy=0, qp=0, A=0, D1=0, D2=0, D3=0, D4=0, a=0, b=0, c=0, root = 0, k = 0;

	qx = qxy.q1;
	qy = qxy.q2;
	qp = qxy.q4;

	if(qp!=1)
		A=1/(1-qp*qp);
	else
		A=0;

	D1 = A*(d_phi*qx*qx+d_ortho*qy*qy);
	D2 = A*(qx*qy*(d_phi-d_ortho));
	D3 = D2;
	D4 = A*(d_phi*qy*qy+d_ortho*qx*qx);

	a = sq(D1*omega_f.x+D2*omega_f.y)+sq(D3*omega_f.x+D4*omega_f.y);
	b = -2*(art_torques.x*(D1*omega_f.x+D2*omega_f.y)+art_torques.y*(D3*omega_f.x+D4*omega_f.y));
	c = art_torques.x*art_torques.x+art_torques.y*art_torques.y-torque_xy_max*torque_xy_max;

	root = b*b-4*a*c;

	if(root < 0 || a == 0)
		return 1.0f;

	if(root > b*b)
		k = (-b+safe_sqrt(root))/(2*a);
	else
		k = (-b-safe_sqrt(root))/(2*a);

	if (k < 0 || k > 1)
		return 1;

	return k;
}

/*
*	Computes the gain that allows to saturate the z control torque
*/
float compute_kz(Vector3<float> art_torques,Vector3<float> omega_f,float d_z)
{
	float T_z = 0, wz = 0, a=0, b=0, c=0, root=0, k=0;

	T_z = art_torques.z;
	wz = omega_f.z;

	a = d_z*wz*d_z*wz;
	b = -2*T_z*d_z*wz;
	c = T_z*T_z-torque_z_max*torque_z_max;

	root = b*b-4*a*c;

	if(root < 0 || a ==0)
		return 1;

	if(root > b*b)
		k = (-b+safe_sqrt(root))/(2*a);
	else
		k = (-b-safe_sqrt(root))/(2*a);

	if (k < 0 || k > 1)
		return 1;

	//hal.console->printf_P(PSTR("k_z: [%.7f]\r\n"),k);
	return k;
}	

/*
*	Computes the damping matrix
*/
Matrix3<float> compute_D_matrix(Quaternion_D qxy, Quaternion_D qz, Vector3<float> omega_f, Vector3<float> art_torques)
{
	float qx=0, qy=0, qzz=0, qp=0, qw=0, wx=0, wy=0, wz=0, d_phi=0, d_z=0, k_xy=0, k_z=0,d1=0,d2=0,d3=0,d4=0,d5=0,d6=0,d7=0,d8=0,d9=0;
	float A=0, B=0;

	qx = qxy.q1;
	qy = qxy.q2;
	qzz = qz.q3;
	qp = qxy.q4;
	qw = qz.q4;

	wx = omega_f.x;
	wy = omega_f.y;
	wz = omega_f.z;

	d_phi = compute_damping_phi(qxy,qz,omega_f,compute_phi(qp),compute_theta(qw));
	d_z = compute_damping_z(qxy,qz,omega_f,compute_phi(qp),compute_theta(qw));

	k_xy = compute_kxy(art_torques,omega_f,qxy,d_phi);
	k_z = compute_kz(art_torques,omega_f,d_z);


	if(qp!=1){
		A = k_xy*d_phi/(1-qp*qp);
		B = k_xy*d_ortho/(1-qp*qp);
	}else{
		A = 0;
		B = 0;
	}

	d1 = A*qx*qx+B*qy*qy;
	d2 = qx*qy*(A-B);
	d4 = d2;
	d5 = A*qy*qy+B*qx*qx;
	d9 = k_z*d_z;

	

	Matrix3<float> ret(d1,d2,d3,d4,d5,d6,d7,d8,d9);

	return ret;

}

/*
*	Implements the fast and saturating attitude controller
*/
Vector3<float> fast_and_saturating_controller(Quaternion_D current_att, Quaternion_D desired_att, Vector3<float> omega_f)
{
	Quaternion_D qe,qz,qxy;
	Vector3<float> T;
	Matrix3<float> D;

	qe = mult_quat(current_att.conjugate(),desired_att); // attitude error
	qe.sign_l();
	qz = qe.get_z();
	qxy = mult_quat_inv(qe,qz);

	//hal.console->printf_P(PSTR("qe:[%.7f,%.7f,%.7f,%.7f]\r\n"),qe.q1,qe.q2,qe.q3,qe.q4);
	//hal.console->printf_P(PSTR("qz:[%.2f,%.2f,%.7f,%.7f]\r\n"),qz.q1,qz.q2,qz.q3,qz.q4);
	//hal.console->printf_P(PSTR("qxy:[%.7f,%.7f,%.2f,%.7f]\r\n"),qxy.q1,qxy.q2,qxy.q3,qxy.q4);
  	T = compute_art_torques(qxy,qz,compute_phi(qxy.q4),compute_theta(qz.q4));
  	D = compute_D_matrix(qxy,qz,omega_f,T);



  	return T-D*omega_f;
}

/*
*
*/
float map_f(float x, float in_min, float in_max, float out_min, float out_max)
{
	if(x > in_max)
		x = in_max;
	if(x < in_min)
		x = in_min;

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	float ret = 0;

	if(x > in_max)
		x = in_max;
	if(x < in_min)
		x = in_min;

  	ret = round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);

  	if (ret > out_max)
  		ret = out_max;
  	if (ret < out_min)
  		ret = out_min;

  	return ret;
}

/*
*	Converts from Thrust + Torques to motor inputs
*/
void to_motors(float Thrust, Vector3<float> torques, uint16_t * u1,uint16_t * u2,uint16_t * u3,uint16_t * u4)
{
	float u1f = 0, u2f = 0, u3f = 0, u4f = 0;
	float fail_factor = 1;

	u1f = 0.5/(arm*c_T)*torques.y*fail_factor + 0.25/(c_D)*torques.z*fail_factor + 0.25/(c_T)*Thrust;
	u2f = -0.5/(arm*c_T)*torques.y*fail_factor + 0.25/(c_D)*torques.z*fail_factor + 0.25/(c_T)*Thrust;
	u3f = 0.5/(arm*c_T)*torques.x*fail_factor - 0.25/(c_D)*torques.z*fail_factor + 0.25/(c_T)*Thrust;
	u4f = -0.5/(arm*c_T)*torques.x*fail_factor - 0.25/(c_D)*torques.z*fail_factor + 0.25/(c_T)*Thrust;

	if(u1f > 1000)
		u1f = 1000;
	else if (u1f < 0)
		u1f = 0;

	if(u2f > 1000)
		u2f = 1000;
	else if (u2f < 0)
		u2f = 0;


	if(u3f > 1000)
		u3f = 1000;
	else if (u3f < 0)
		u3f = 0;

	if(u4f > 1000)
		u4f = 1000;
	else if (u4f < 0)
		u4f = 0;


	*u1 = (uint16_t) u1f+1000; // to radio values
	*u2 = (uint16_t) u2f+1000; // to radio values
	*u3 = (uint16_t) u3f+1000; // to radio values
	*u4 = (uint16_t) u4f+1000; // to radio values
}
