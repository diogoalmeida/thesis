/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Copyright 2014 Diogo Almeida - diogormalmeida@gmail.com, all rights reserved.

// Implements functions required for the fast and saturating controller

// Required variables


#include <quaternion_diogo.h>
#include <math.h>
#include <AP_Math.h>

#define phi_multiplier 1
#define torque_multiplier 4

#define phi_low 10*phi_multiplier*PI/180
#define theta_low 15*PI/180
#define c_phi 0.817/(torque_multiplier*phi_multiplier)//0.817
#define c_theta 0.109//0.109
#define delta_phi 5*PI/180
#define delta_theta 5*PI/180
#define small_delta_phi 0.1999
#define small_delta_z 0.0961
#define d_ortho small_delta_phi
#define phi_up 175*PI/180
#define theta_up 175*PI/180
#define r_phi 0.75/2
#define r_theta 0.75
#define v_phi_max 1.425
#define v_theta_max 0.624
#define v_phi 0.1
#define v_theta 0.1
#define J_x 0.0085
#define J_z 0.014

#define arm 0.28 // m
#define max_motor_thrust 1.095 //kg
#define c_T max_motor_thrust/1000 // max input -> max thrust [N/s]
#define c_D c_T/3 // ??? [N.m/s]

#define torque_xy_max 0.15/torque_multiplier//0.15// ??? [N.m]
#define torque_z_max 0.03//0.03 // ??? [N.m]

// RC params
#define rT_min 1032
#define rT_max 1986
#define T_min 0.5
#define T_max 3.0

#define rR_min 1032
#define rR_max 1986
#define R_min  -45
#define R_max  45

#define rP_min 1032
#define rP_max 1986
#define P_min  -45
#define P_max  45

#define rY_min 1032
#define rY_max 1986
#define Y_min  -180
#define Y_max  180






float lambda_f(float up,float low,float val);
float integral_lambda_f(float up, float low, float val);
float xi_f(float up, float low, float f1, float f2, float val);
float double_xi_f(float up1, float up2, float low1, float low2,float f1, float f2,float val);

float compute_phi(float qp);
float compute_theta(float qw);

Vector3<float> compute_art_torques(Quaternion_D qxy, Quaternion_D qz,float phi, float theta);

float compute_phi_dot(Quaternion_D qxy,Vector3<float> omega_f);
float compute_theta_dot(Quaternion_D qxy, Quaternion_D qz, Vector3<float> omega_f);

float compute_switch_curve_phi(float phi);
float compute_switch_curve_theta(float theta);

float compute_torque_phi(Quaternion_D qxy, float phi, float theta,Vector3<float> omega_f);
float compute_torque_z(Quaternion_D qxy, Quaternion_D qz,float theta,Vector3<float> omega_f);

float compute_acc_damping_phi(float phi_dot, float T_phi);
float compute_dec_damping_phi(float phi_dot, float T_phi);
float compute_star_damping_phi(float phi_dot,float dec_phi, float acc_phi, float switch_phi);
float compute_damping_phi(Quaternion_D qxy, Vector3<float> omega_f,float phi,float theta);

float compute_acc_damping_z(float theta_dot_hole, float T_z);
float compute_dec_damping_z(float theta_dot_hole, float T_z);
float compute_star_damping_z(float theta_dot,float theta_dot_hole,float dec_theta, float acc_theta, float switch_theta);
float compute_damping_z(Quaternion_D qxy,Quaternion_D qz, Vector3<float> omega_f,float phi,float theta);

float compute_kxy(Vector3<float> art_torques,Vector3<float> omega_f, Quaternion_D qxy,float d_phi);
float compute_kz(Vector3<float> art_torques,Vector3<float> omega_f,float d_z);

Matrix3<float> compute_D_matrix(Quaternion_D qxy, Quaternion_D qz, Vector3<float> omega_f, Vector3<float> art_torques);

Vector3<float> fast_and_saturating_controller(Quaternion_D current_att, Quaternion_D desired_att, Vector3<float> omega_f);

float map(float x, float in_min, float in_max, float out_min, float out_max);

void to_motors(float Thrust, Vector3<float> torques, uint16_t * u1,uint16_t * u2,uint16_t * u3,uint16_t * u4);