/* Include files */

#include <stddef.h>
#include "blas.h"
#include "Thrust_Direction_quad_sfun.h"
#include "c2_Thrust_Direction_quad.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Thrust_Direction_quad_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[21] = { "q", "q_d", "q_e", "q_z",
  "q_xy", "w_x", "w_y", "w_z", "Torque_phi", "Torque_field", "nargin", "nargout",
  "Orientation", "Desired_Orientation", "Angular_Vel", "last_phi", "Torques",
  "phi", "phi_low", "c_phi", "phi_up" };

static const char * c2_b_debug_family_names[7] = { "roll", "pitch", "yaw",
  "nargin", "nargout", "Orientation", "quat" };

static const char * c2_c_debug_family_names[4] = { "nargin", "nargout", "q_i",
  "q_o" };

static const char * c2_d_debug_family_names[6] = { "W", "q_o", "nargin",
  "nargout", "q_1", "q_2" };

static const char * c2_e_debug_family_names[4] = { "nargin", "nargout", "u",
  "ret" };

static const char * c2_f_debug_family_names[4] = { "nargin", "nargout", "q",
  "q_z" };

static const char * c2_g_debug_family_names[6] = { "W", "q_o", "nargin",
  "nargout", "q", "q_inv" };

/* Function Declarations */
static void initialize_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void initialize_params_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void enable_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void disable_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void set_sim_state_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance, const mxArray *c2_st);
static void finalize_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void sf_c2_Thrust_Direction_quad(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance);
static void c2_chartstep_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void initSimStructsc2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void registerMessagesc2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void c2_angle_to_quat(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_Orientation[3], real_T c2_quat[4]);
static void c2_quat_mult(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  real_T c2_q_1[4], real_T c2_q_2[4], real_T c2_q_o[4]);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_phi_up, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_Torques, const char_T *c2_identifier, real_T
  c2_y[3]);
static void c2_d_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_e_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_f_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_g_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_h_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16]);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[148]);
static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[148]);
static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[148]);
static void c2_eml_scalar_eg(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance);
static real_T c2_mpower(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  real_T c2_a);
static void c2_b_eml_scalar_eg(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance);
static void c2_eml_error(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void c2_realmin(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void c2_eps(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance);
static void c2_eml_matlab_zgetrf(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_b_A[16], int32_T c2_ipiv[4],
  int32_T *c2_info);
static void c2_check_forloop_overflow_error
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance, boolean_T
   c2_overflow);
static void c2_eml_xger(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0,
  real_T c2_A[16], int32_T c2_ia0, real_T c2_b_A[16]);
static void c2_eml_warning(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance);
static void c2_eml_xtrsm(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  real_T c2_A[16], real_T c2_B[4], real_T c2_b_B[4]);
static void c2_below_threshold(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance);
static void c2_c_eml_scalar_eg(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance);
static void c2_b_eml_xtrsm(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[4], real_T c2_b_B[4]);
static void c2_b_eml_error(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_i_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_j_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_Thrust_Direction_quad, const
  char_T *c2_identifier);
static uint8_T c2_k_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_eml_matlab_zgetrf(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_A[16], int32_T c2_ipiv[4], int32_T *c2_info);
static void c2_b_eml_xger(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0,
  int32_T c2_iy0, real_T c2_A[16], int32_T c2_ia0);
static void c2_c_eml_xtrsm(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[4]);
static void c2_d_eml_xtrsm(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[4]);
static real_T c2_get_J_x(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b);
static void c2_set_J_x(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c);
static real_T *c2_access_J_x(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_J_y(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b);
static void c2_set_J_y(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c);
static real_T *c2_access_J_y(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_J_z(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b);
static void c2_set_J_z(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c);
static real_T *c2_access_J_z(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_brake_torque(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static void c2_set_brake_torque(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c);
static real_T *c2_access_brake_torque(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_c_phi(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static void c2_set_c_phi(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c);
static real_T *c2_access_c_phi(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_d(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b);
static void c2_set_d(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
                     uint32_T c2_b, real_T c2_c);
static real_T *c2_access_d(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_d_l(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b);
static void c2_set_d_l(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c);
static real_T *c2_access_d_l(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_d_z(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b);
static void c2_set_d_z(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c);
static real_T *c2_access_d_z(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_delta_phi(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static void c2_set_delta_phi(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c);
static real_T *c2_access_delta_phi(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_phi_low(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static void c2_set_phi_low(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c);
static real_T *c2_access_phi_low(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_phi_up(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static void c2_set_phi_up(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c);
static real_T *c2_access_phi_up(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_r(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b);
static void c2_set_r(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
                     uint32_T c2_b, real_T c2_c);
static real_T *c2_access_r(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_t_s(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b);
static void c2_set_t_s(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c);
static real_T *c2_access_t_s(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_torque_xy(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static void c2_set_torque_xy(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c);
static real_T *c2_access_torque_xy(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_torque_z(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static void c2_set_torque_z(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c);
static real_T *c2_access_torque_z(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_v_1(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b);
static void c2_set_v_1(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c);
static real_T *c2_access_v_1(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static real_T c2_get_v_2(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b);
static void c2_set_v_2(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c);
static real_T *c2_access_v_2(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b);
static void init_dsm_address_info(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_Thrust_Direction_quad = 0U;
}

static void initialize_params_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
}

static void enable_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[3];
  const mxArray *c2_b_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  uint8_T c2_b_hoistedGlobal;
  uint8_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T *c2_phi;
  real_T (*c2_Torques)[3];
  c2_phi = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_Torques = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(3), FALSE);
  for (c2_i0 = 0; c2_i0 < 3; c2_i0++) {
    c2_u[c2_i0] = (*c2_Torques)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal = *c2_phi;
  c2_b_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_b_hoistedGlobal = chartInstance->c2_is_active_c2_Thrust_Direction_quad;
  c2_c_u = c2_b_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[3];
  int32_T c2_i1;
  real_T *c2_phi;
  real_T (*c2_Torques)[3];
  c2_phi = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_Torques = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
                        "Torques", c2_dv0);
  for (c2_i1 = 0; c2_i1 < 3; c2_i1++) {
    (*c2_Torques)[c2_i1] = c2_dv0[c2_i1];
  }

  *c2_phi = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
    "phi");
  chartInstance->c2_is_active_c2_Thrust_Direction_quad = c2_j_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
     "is_active_c2_Thrust_Direction_quad");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_Thrust_Direction_quad(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
}

static void sf_c2_Thrust_Direction_quad(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance)
{
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  int32_T c2_i5;
  real_T *c2_last_phi;
  real_T *c2_phi;
  real_T (*c2_Angular_Vel)[3];
  real_T (*c2_Desired_Orientation)[3];
  real_T (*c2_Torques)[3];
  real_T (*c2_Orientation)[3];
  c2_phi = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_last_phi = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_Angular_Vel = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c2_Desired_Orientation = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S,
    1);
  c2_Torques = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_Orientation = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i2 = 0; c2_i2 < 3; c2_i2++) {
    _SFD_DATA_RANGE_CHECK((*c2_Orientation)[c2_i2], 0U);
  }

  for (c2_i3 = 0; c2_i3 < 3; c2_i3++) {
    _SFD_DATA_RANGE_CHECK((*c2_Torques)[c2_i3], 1U);
  }

  for (c2_i4 = 0; c2_i4 < 3; c2_i4++) {
    _SFD_DATA_RANGE_CHECK((*c2_Desired_Orientation)[c2_i4], 2U);
  }

  for (c2_i5 = 0; c2_i5 < 3; c2_i5++) {
    _SFD_DATA_RANGE_CHECK((*c2_Angular_Vel)[c2_i5], 3U);
  }

  _SFD_DATA_RANGE_CHECK(*c2_last_phi, 4U);
  _SFD_DATA_RANGE_CHECK(*c2_phi, 5U);
  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_Thrust_Direction_quad(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_Thrust_Direction_quadMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  int32_T c2_i6;
  real_T c2_Orientation[3];
  int32_T c2_i7;
  real_T c2_Desired_Orientation[3];
  int32_T c2_i8;
  real_T c2_Angular_Vel[3];
  real_T c2_last_phi;
  uint32_T c2_debug_family_var_map[21];
  real_T c2_q[4];
  real_T c2_q_d[4];
  real_T c2_q_e[4];
  real_T c2_q_z[4];
  real_T c2_q_xy[4];
  real_T c2_w_x;
  real_T c2_w_y;
  real_T c2_w_z;
  real_T c2_Torque_phi;
  real_T c2_Torque_field[3];
  real_T c2_b_Torque_field;
  real_T c2_nargin = 4.0;
  real_T c2_nargout = 2.0;
  real_T c2_Torques[3];
  real_T c2_phi;
  int32_T c2_i9;
  real_T c2_b_Orientation[3];
  real_T c2_dv1[4];
  int32_T c2_i10;
  int32_T c2_i11;
  real_T c2_b_Desired_Orientation[3];
  real_T c2_dv2[4];
  int32_T c2_i12;
  int32_T c2_i13;
  real_T c2_q_i[4];
  uint32_T c2_b_debug_family_var_map[4];
  real_T c2_b_nargin = 1.0;
  real_T c2_b_nargout = 1.0;
  real_T c2_q_o[4];
  int32_T c2_i14;
  int32_T c2_i15;
  int32_T c2_i16;
  real_T c2_b_q_o[4];
  int32_T c2_i17;
  real_T c2_b_q_d[4];
  real_T c2_dv3[4];
  int32_T c2_i18;
  real_T c2_u;
  real_T c2_c_nargin = 1.0;
  real_T c2_c_nargout = 1.0;
  real_T c2_ret;
  int32_T c2_i19;
  int32_T c2_i20;
  real_T c2_b_q[4];
  real_T c2_d_nargin = 1.0;
  real_T c2_d_nargout = 1.0;
  int32_T c2_i21;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_A;
  real_T c2_B;
  real_T c2_c_x;
  real_T c2_y;
  real_T c2_d_x;
  real_T c2_b_y;
  real_T c2_c_y;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_g_x;
  real_T c2_d_y;
  real_T c2_h_x;
  real_T c2_e_y;
  real_T c2_f_y;
  int32_T c2_i22;
  real_T c2_c_q[4];
  int32_T c2_i23;
  real_T c2_q_inv[4];
  uint32_T c2_c_debug_family_var_map[6];
  real_T c2_W[16];
  real_T c2_c_q_o[4];
  real_T c2_e_nargin = 2.0;
  real_T c2_e_nargout = 1.0;
  int32_T c2_i24;
  real_T c2_c_A[16];
  int32_T c2_i25;
  real_T c2_c_B[4];
  int32_T c2_info;
  int32_T c2_ipiv[4];
  int32_T c2_b_info;
  int32_T c2_c_info;
  int32_T c2_d_info;
  int32_T c2_i26;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_ip;
  real_T c2_temp;
  int32_T c2_i27;
  real_T c2_dv4[16];
  int32_T c2_i28;
  real_T c2_dv5[16];
  int32_T c2_i29;
  real_T c2_dv6[16];
  int32_T c2_i30;
  real_T c2_dv7[16];
  int32_T c2_i31;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_b;
  real_T c2_b_hoistedGlobal;
  real_T c2_a;
  real_T c2_b_b;
  real_T c2_c_hoistedGlobal;
  real_T c2_d_hoistedGlobal;
  real_T c2_b_a;
  real_T c2_c_b;
  real_T c2_e_hoistedGlobal;
  real_T c2_f_hoistedGlobal;
  real_T c2_c_a;
  real_T c2_d_b;
  real_T c2_g_y;
  real_T c2_d_a;
  real_T c2_e_b;
  real_T c2_h_y;
  real_T c2_g_hoistedGlobal;
  real_T c2_d_A;
  real_T c2_d_B;
  real_T c2_k_x;
  real_T c2_i_y;
  real_T c2_l_x;
  real_T c2_j_y;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_e_B;
  real_T c2_k_y;
  real_T c2_l_y;
  real_T c2_m_y;
  real_T c2_e_a;
  real_T c2_f_b;
  real_T c2_b_q_xy[3];
  int32_T c2_i32;
  int32_T c2_i33;
  int32_T c2_i34;
  int32_T c2_i35;
  real_T (*c2_b_Torques)[3];
  real_T *c2_b_last_phi;
  real_T *c2_b_phi;
  real_T (*c2_b_Angular_Vel)[3];
  real_T (*c2_c_Desired_Orientation)[3];
  real_T (*c2_c_Orientation)[3];
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  boolean_T guard4 = FALSE;
  c2_b_phi = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_last_phi = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_Angular_Vel = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c2_c_Desired_Orientation = (real_T (*)[3])ssGetInputPortSignal
    (chartInstance->S, 1);
  c2_b_Torques = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_c_Orientation = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_last_phi;
  for (c2_i6 = 0; c2_i6 < 3; c2_i6++) {
    c2_Orientation[c2_i6] = (*c2_c_Orientation)[c2_i6];
  }

  for (c2_i7 = 0; c2_i7 < 3; c2_i7++) {
    c2_Desired_Orientation[c2_i7] = (*c2_c_Desired_Orientation)[c2_i7];
  }

  for (c2_i8 = 0; c2_i8 < 3; c2_i8++) {
    c2_Angular_Vel[c2_i8] = (*c2_b_Angular_Vel)[c2_i8];
  }

  c2_last_phi = c2_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 21U, 22U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q, 0U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_d, 1U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_e, 2U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_z, 3U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_xy, 4U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_w_x, 5U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_w_y, 6U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_w_z, 7U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Torque_phi, 8U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Torque_field, MAX_uint32_T,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_Torque_field, MAX_uint32_T,
    c2_sf_marshallOut, c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 10U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 11U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_Orientation, 12U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_Desired_Orientation, 13U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_Angular_Vel, 14U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_last_phi, 15U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Torques, 16U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_phi, 17U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_phi_low_address, 18U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_c_phi_address, 19U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_phi_up_address, 20U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 9);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 18);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 19);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 20);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
  for (c2_i9 = 0; c2_i9 < 3; c2_i9++) {
    c2_b_Orientation[c2_i9] = c2_Orientation[c2_i9];
  }

  c2_angle_to_quat(chartInstance, c2_b_Orientation, c2_dv1);
  for (c2_i10 = 0; c2_i10 < 4; c2_i10++) {
    c2_q[c2_i10] = c2_dv1[c2_i10];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 24);
  for (c2_i11 = 0; c2_i11 < 3; c2_i11++) {
    c2_b_Desired_Orientation[c2_i11] = c2_Desired_Orientation[c2_i11];
  }

  c2_angle_to_quat(chartInstance, c2_b_Desired_Orientation, c2_dv2);
  for (c2_i12 = 0; c2_i12 < 4; c2_i12++) {
    c2_q_d[c2_i12] = c2_dv2[c2_i12];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 26);
  for (c2_i13 = 0; c2_i13 < 4; c2_i13++) {
    c2_q_i[c2_i13] = c2_q[c2_i13];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_c_debug_family_names,
    c2_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_i, 2U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_o, 3U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 6);
  for (c2_i14 = 0; c2_i14 < 4; c2_i14++) {
    c2_q_o[c2_i14] = 0.0;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 7);
  for (c2_i15 = 0; c2_i15 < 3; c2_i15++) {
    c2_q_o[c2_i15] = -c2_q_i[c2_i15];
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 8);
  c2_q_o[3] = c2_q_i[3];
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, -8);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i16 = 0; c2_i16 < 4; c2_i16++) {
    c2_b_q_o[c2_i16] = c2_q_o[c2_i16];
  }

  for (c2_i17 = 0; c2_i17 < 4; c2_i17++) {
    c2_b_q_d[c2_i17] = c2_q_d[c2_i17];
  }

  c2_quat_mult(chartInstance, c2_b_q_o, c2_b_q_d, c2_dv3);
  for (c2_i18 = 0; c2_i18 < 4; c2_i18++) {
    c2_q_e[c2_i18] = c2_dv3[c2_i18];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_u = c2_q_e[3];
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_e_debug_family_names,
    c2_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_u, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_ret, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_SCRIPT_FCN(3, 0);
  _SFD_SCRIPT_CALL(3U, chartInstance->c2_sfEvent, 4);
  if (CV_SCRIPT_IF(3, 0, c2_u >= 0.0)) {
    _SFD_SCRIPT_CALL(3U, chartInstance->c2_sfEvent, 5);
    c2_ret = 1.0;
  } else {
    _SFD_SCRIPT_CALL(3U, chartInstance->c2_sfEvent, 7);
    c2_ret = 0.0;
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c2_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i19 = 0; c2_i19 < 4; c2_i19++) {
    c2_q_e[c2_i19] *= c2_ret;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 30);
  for (c2_i20 = 0; c2_i20 < 4; c2_i20++) {
    c2_b_q[c2_i20] = c2_q_e[c2_i20];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_f_debug_family_names,
    c2_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_q, 2U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_z, 3U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  CV_SCRIPT_FCN(4, 0);
  _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 5);
  for (c2_i21 = 0; c2_i21 < 4; c2_i21++) {
    c2_q_z[c2_i21] = 0.0;
  }

  _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 6);
  c2_x = c2_mpower(chartInstance, c2_b_q[2]) + c2_mpower(chartInstance, c2_b_q[3]);
  c2_b_x = c2_x;
  if (c2_b_x < 0.0) {
    c2_eml_error(chartInstance);
  }

  c2_b_x = muDoubleScalarSqrt(c2_b_x);
  c2_A = c2_b_q[2];
  c2_B = c2_b_x;
  c2_c_x = c2_A;
  c2_y = c2_B;
  c2_d_x = c2_c_x;
  c2_b_y = c2_y;
  c2_c_y = c2_d_x / c2_b_y;
  c2_q_z[2] = c2_c_y;
  _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 7);
  c2_e_x = c2_mpower(chartInstance, c2_b_q[2]) + c2_mpower(chartInstance,
    c2_b_q[3]);
  c2_f_x = c2_e_x;
  if (c2_f_x < 0.0) {
    c2_eml_error(chartInstance);
  }

  c2_f_x = muDoubleScalarSqrt(c2_f_x);
  c2_b_A = c2_b_q[3];
  c2_b_B = c2_f_x;
  c2_g_x = c2_b_A;
  c2_d_y = c2_b_B;
  c2_h_x = c2_g_x;
  c2_e_y = c2_d_y;
  c2_f_y = c2_h_x / c2_e_y;
  c2_q_z[3] = c2_f_y;
  _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  for (c2_i22 = 0; c2_i22 < 4; c2_i22++) {
    c2_c_q[c2_i22] = c2_q[c2_i22];
  }

  for (c2_i23 = 0; c2_i23 < 4; c2_i23++) {
    c2_q_inv[c2_i23] = c2_q_z[c2_i23];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 7U, c2_g_debug_family_names,
    c2_c_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_W, 0U, c2_f_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_c_q_o, MAX_uint32_T,
    c2_e_sf_marshallOut, c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_e_nargin, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_e_nargout, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_c_q, 4U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_inv, 5U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_xy, MAX_uint32_T,
    c2_d_sf_marshallOut, c2_d_sf_marshallIn);
  CV_SCRIPT_FCN(5, 0);
  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 4);
  c2_W[0] = c2_q_inv[3];
  c2_W[4] = c2_q_inv[2];
  c2_W[8] = -c2_q_inv[1];
  c2_W[12] = c2_q_inv[0];
  c2_W[1] = -c2_q_inv[2];
  c2_W[5] = c2_q_inv[3];
  c2_W[9] = c2_q_inv[0];
  c2_W[13] = c2_q_inv[1];
  c2_W[2] = c2_q_inv[1];
  c2_W[6] = -c2_q_inv[0];
  c2_W[10] = c2_q_inv[3];
  c2_W[14] = c2_q_inv[2];
  c2_W[3] = -c2_q_inv[0];
  c2_W[7] = -c2_q_inv[1];
  c2_W[11] = -c2_q_inv[2];
  c2_W[15] = c2_q_inv[3];
  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 9);
  for (c2_i24 = 0; c2_i24 < 16; c2_i24++) {
    c2_c_A[c2_i24] = c2_W[c2_i24];
  }

  for (c2_i25 = 0; c2_i25 < 4; c2_i25++) {
    c2_c_B[c2_i25] = c2_c_q[c2_i25];
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_c_A, c2_ipiv, &c2_info);
  c2_b_info = c2_info;
  c2_c_info = c2_b_info;
  c2_d_info = c2_c_info;
  if (c2_d_info > 0) {
    c2_eml_warning(chartInstance);
  }

  c2_eml_scalar_eg(chartInstance);
  for (c2_i26 = 0; c2_i26 < 4; c2_i26++) {
    c2_c_q_o[c2_i26] = c2_c_B[c2_i26];
  }

  for (c2_i = 1; c2_i < 5; c2_i++) {
    c2_b_i = c2_i;
    if (c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_i), 1, 4, 1, 0) - 1] != c2_b_i) {
      c2_ip = c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 4, 1, 0) - 1];
      c2_temp = c2_c_q_o[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 4, 1, 0) - 1];
      c2_c_q_o[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_i), 1, 4, 1, 0) - 1] = c2_c_q_o[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_ip), 1, 4, 1, 0) - 1];
      c2_c_q_o[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_ip), 1, 4, 1, 0) - 1] = c2_temp;
    }
  }

  for (c2_i27 = 0; c2_i27 < 16; c2_i27++) {
    c2_dv4[c2_i27] = c2_c_A[c2_i27];
  }

  for (c2_i28 = 0; c2_i28 < 16; c2_i28++) {
    c2_dv5[c2_i28] = c2_dv4[c2_i28];
  }

  c2_c_eml_xtrsm(chartInstance, c2_dv5, c2_c_q_o);
  for (c2_i29 = 0; c2_i29 < 16; c2_i29++) {
    c2_dv6[c2_i29] = c2_c_A[c2_i29];
  }

  for (c2_i30 = 0; c2_i30 < 16; c2_i30++) {
    c2_dv7[c2_i30] = c2_dv6[c2_i30];
  }

  c2_d_eml_xtrsm(chartInstance, c2_dv7, c2_c_q_o);
  _SFD_SYMBOL_SWITCH(1U, 1U);
  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 11);
  for (c2_i31 = 0; c2_i31 < 4; c2_i31++) {
    c2_q_xy[c2_i31] = c2_c_q_o[c2_i31];
  }

  _SFD_SYMBOL_SWITCH(1U, 6U);
  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, -11);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 35);
  c2_i_x = c2_q_xy[3];
  c2_j_x = c2_i_x;
  guard4 = FALSE;
  if (c2_j_x < -1.0) {
    guard4 = TRUE;
  } else {
    if (1.0 < c2_j_x) {
      guard4 = TRUE;
    }
  }

  if (guard4 == TRUE) {
    c2_b_eml_error(chartInstance);
  }

  c2_j_x = muDoubleScalarAcos(c2_j_x);
  c2_b = c2_j_x;
  c2_phi = 2.0 * c2_b;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 36);
  c2_w_x = c2_Angular_Vel[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 37);
  c2_w_y = c2_Angular_Vel[1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 38);
  c2_w_z = c2_Angular_Vel[2];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 42);
  c2_Torque_phi = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 44);
  guard1 = FALSE;
  if (CV_EML_COND(0, 1, 0, c2_phi >= 0.0)) {
    if (CV_EML_COND(0, 1, 1, c2_phi <= c2_get_phi_low(chartInstance, 0))) {
      CV_EML_MCDC(0, 1, 0, TRUE);
      CV_EML_IF(0, 1, 0, TRUE);
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 46);
      c2_b_hoistedGlobal = c2_get_c_phi(chartInstance, 0);
      c2_a = c2_b_hoistedGlobal;
      c2_b_b = c2_phi;
      c2_Torque_phi = c2_a * c2_b_b;
    } else {
      guard1 = TRUE;
    }
  } else {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    CV_EML_MCDC(0, 1, 0, FALSE);
    CV_EML_IF(0, 1, 0, FALSE);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 48);
    guard2 = FALSE;
    if (CV_EML_COND(0, 1, 2, c2_phi > c2_get_phi_low(chartInstance, 0))) {
      if (CV_EML_COND(0, 1, 3, c2_phi <= c2_get_phi_up(chartInstance, 0))) {
        CV_EML_MCDC(0, 1, 1, TRUE);
        CV_EML_IF(0, 1, 1, TRUE);
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 50);
        c2_c_hoistedGlobal = c2_get_c_phi(chartInstance, 0);
        c2_d_hoistedGlobal = c2_get_phi_low(chartInstance, 0);
        c2_b_a = c2_c_hoistedGlobal;
        c2_c_b = c2_d_hoistedGlobal;
        c2_Torque_phi = c2_b_a * c2_c_b;
      } else {
        guard2 = TRUE;
      }
    } else {
      guard2 = TRUE;
    }

    if (guard2 == TRUE) {
      CV_EML_MCDC(0, 1, 1, FALSE);
      CV_EML_IF(0, 1, 1, FALSE);
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 52);
      guard3 = FALSE;
      if (CV_EML_COND(0, 1, 4, c2_phi > c2_get_phi_up(chartInstance, 0))) {
        if (CV_EML_COND(0, 1, 5, c2_phi <= 3.1415926535897931)) {
          CV_EML_MCDC(0, 1, 2, TRUE);
          CV_EML_IF(0, 1, 2, TRUE);
          _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 54);
          c2_e_hoistedGlobal = c2_get_c_phi(chartInstance, 0);
          c2_f_hoistedGlobal = c2_get_phi_low(chartInstance, 0);
          c2_c_a = c2_e_hoistedGlobal;
          c2_d_b = c2_f_hoistedGlobal;
          c2_g_y = c2_c_a * c2_d_b;
          c2_d_a = c2_g_y;
          c2_e_b = 3.1415926535897931 - c2_phi;
          c2_h_y = c2_d_a * c2_e_b;
          c2_g_hoistedGlobal = c2_get_phi_up(chartInstance, 0);
          c2_d_A = c2_h_y;
          c2_d_B = 3.1415926535897931 - c2_g_hoistedGlobal;
          c2_k_x = c2_d_A;
          c2_i_y = c2_d_B;
          c2_l_x = c2_k_x;
          c2_j_y = c2_i_y;
          c2_Torque_phi = c2_l_x / c2_j_y;
        } else {
          guard3 = TRUE;
        }
      } else {
        guard3 = TRUE;
      }

      if (guard3 == TRUE) {
        CV_EML_MCDC(0, 1, 2, FALSE);
        CV_EML_IF(0, 1, 2, FALSE);
      }
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 61);
  if (CV_EML_IF(0, 1, 3, c2_phi != 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 62);
    c2_m_x = 1.0 - c2_mpower(chartInstance, c2_q_xy[3]);
    c2_n_x = c2_m_x;
    if (c2_n_x < 0.0) {
      c2_eml_error(chartInstance);
    }

    c2_n_x = muDoubleScalarSqrt(c2_n_x);
    c2_e_B = c2_n_x;
    c2_k_y = c2_e_B;
    c2_l_y = c2_k_y;
    c2_m_y = 1.0 / c2_l_y;
    c2_e_a = c2_Torque_phi;
    c2_f_b = c2_m_y;
    c2_b_Torque_field = c2_e_a * c2_f_b;
    _SFD_SYMBOL_SWITCH(9U, 10U);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 63);
    c2_b_q_xy[0] = c2_q_xy[0];
    c2_b_q_xy[1] = c2_q_xy[1];
    c2_b_q_xy[2] = 0.0;
    for (c2_i32 = 0; c2_i32 < 3; c2_i32++) {
      c2_Torque_field[c2_i32] = c2_b_Torque_field * c2_b_q_xy[c2_i32];
    }

    _SFD_SYMBOL_SWITCH(9U, 9U);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 65);
    for (c2_i33 = 0; c2_i33 < 3; c2_i33++) {
      c2_Torque_field[c2_i33] = 0.0;
    }

    _SFD_SYMBOL_SWITCH(9U, 9U);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 199U);
  for (c2_i34 = 0; c2_i34 < 3; c2_i34++) {
    c2_Torques[c2_i34] = c2_Torque_field[c2_i34];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -199);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i35 = 0; c2_i35 < 3; c2_i35++) {
    (*c2_b_Torques)[c2_i35] = c2_Torques[c2_i35];
  }

  *c2_b_phi = c2_phi;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
}

static void registerMessagesc2_Thrust_Direction_quad
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
}

static void c2_angle_to_quat(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_Orientation[3], real_T c2_quat[4])
{
  uint32_T c2_debug_family_var_map[7];
  real_T c2_roll;
  real_T c2_pitch;
  real_T c2_yaw;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i36;
  real_T c2_A;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_y;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_b_A;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_b_y;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_a;
  real_T c2_b;
  real_T c2_c_y;
  real_T c2_c_A;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_d_y;
  real_T c2_k_x;
  real_T c2_l_x;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_e_y;
  real_T c2_d_A;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_f_y;
  real_T c2_o_x;
  real_T c2_p_x;
  real_T c2_e_A;
  real_T c2_q_x;
  real_T c2_r_x;
  real_T c2_g_y;
  real_T c2_s_x;
  real_T c2_t_x;
  real_T c2_c_a;
  real_T c2_c_b;
  real_T c2_h_y;
  real_T c2_f_A;
  real_T c2_u_x;
  real_T c2_v_x;
  real_T c2_i_y;
  real_T c2_w_x;
  real_T c2_x_x;
  real_T c2_d_a;
  real_T c2_d_b;
  real_T c2_j_y;
  real_T c2_g_A;
  real_T c2_y_x;
  real_T c2_ab_x;
  real_T c2_k_y;
  real_T c2_bb_x;
  real_T c2_cb_x;
  real_T c2_h_A;
  real_T c2_db_x;
  real_T c2_eb_x;
  real_T c2_l_y;
  real_T c2_fb_x;
  real_T c2_gb_x;
  real_T c2_e_a;
  real_T c2_e_b;
  real_T c2_m_y;
  real_T c2_i_A;
  real_T c2_hb_x;
  real_T c2_ib_x;
  real_T c2_n_y;
  real_T c2_jb_x;
  real_T c2_kb_x;
  real_T c2_f_a;
  real_T c2_f_b;
  real_T c2_o_y;
  real_T c2_j_A;
  real_T c2_lb_x;
  real_T c2_mb_x;
  real_T c2_p_y;
  real_T c2_nb_x;
  real_T c2_ob_x;
  real_T c2_k_A;
  real_T c2_pb_x;
  real_T c2_qb_x;
  real_T c2_q_y;
  real_T c2_rb_x;
  real_T c2_sb_x;
  real_T c2_g_a;
  real_T c2_g_b;
  real_T c2_r_y;
  real_T c2_l_A;
  real_T c2_tb_x;
  real_T c2_ub_x;
  real_T c2_s_y;
  real_T c2_vb_x;
  real_T c2_wb_x;
  real_T c2_h_a;
  real_T c2_h_b;
  real_T c2_t_y;
  real_T c2_m_A;
  real_T c2_xb_x;
  real_T c2_yb_x;
  real_T c2_u_y;
  real_T c2_ac_x;
  real_T c2_bc_x;
  real_T c2_n_A;
  real_T c2_cc_x;
  real_T c2_dc_x;
  real_T c2_v_y;
  real_T c2_ec_x;
  real_T c2_fc_x;
  real_T c2_i_a;
  real_T c2_i_b;
  real_T c2_w_y;
  real_T c2_o_A;
  real_T c2_gc_x;
  real_T c2_hc_x;
  real_T c2_x_y;
  real_T c2_ic_x;
  real_T c2_jc_x;
  real_T c2_j_a;
  real_T c2_j_b;
  real_T c2_y_y;
  real_T c2_p_A;
  real_T c2_kc_x;
  real_T c2_lc_x;
  real_T c2_ab_y;
  real_T c2_mc_x;
  real_T c2_nc_x;
  real_T c2_q_A;
  real_T c2_oc_x;
  real_T c2_pc_x;
  real_T c2_bb_y;
  real_T c2_qc_x;
  real_T c2_rc_x;
  real_T c2_k_a;
  real_T c2_k_b;
  real_T c2_cb_y;
  real_T c2_r_A;
  real_T c2_sc_x;
  real_T c2_tc_x;
  real_T c2_db_y;
  real_T c2_uc_x;
  real_T c2_vc_x;
  real_T c2_l_a;
  real_T c2_l_b;
  real_T c2_eb_y;
  real_T c2_s_A;
  real_T c2_wc_x;
  real_T c2_xc_x;
  real_T c2_fb_y;
  real_T c2_yc_x;
  real_T c2_ad_x;
  real_T c2_t_A;
  real_T c2_bd_x;
  real_T c2_cd_x;
  real_T c2_gb_y;
  real_T c2_dd_x;
  real_T c2_ed_x;
  real_T c2_m_a;
  real_T c2_m_b;
  real_T c2_hb_y;
  real_T c2_u_A;
  real_T c2_fd_x;
  real_T c2_gd_x;
  real_T c2_ib_y;
  real_T c2_hd_x;
  real_T c2_id_x;
  real_T c2_n_a;
  real_T c2_n_b;
  real_T c2_jb_y;
  real_T c2_v_A;
  real_T c2_jd_x;
  real_T c2_kd_x;
  real_T c2_kb_y;
  real_T c2_ld_x;
  real_T c2_md_x;
  real_T c2_w_A;
  real_T c2_nd_x;
  real_T c2_od_x;
  real_T c2_lb_y;
  real_T c2_pd_x;
  real_T c2_qd_x;
  real_T c2_o_a;
  real_T c2_o_b;
  real_T c2_mb_y;
  real_T c2_x_A;
  real_T c2_rd_x;
  real_T c2_sd_x;
  real_T c2_nb_y;
  real_T c2_td_x;
  real_T c2_ud_x;
  real_T c2_p_a;
  real_T c2_p_b;
  real_T c2_ob_y;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 7U, c2_b_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_roll, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_pitch, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_yaw, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Orientation, 5U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_quat, 6U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_roll = c2_Orientation[0];
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 8);
  c2_pitch = c2_Orientation[1];
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 9);
  c2_yaw = c2_Orientation[2];
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 11);
  for (c2_i36 = 0; c2_i36 < 4; c2_i36++) {
    c2_quat[c2_i36] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 13);
  c2_A = c2_roll;
  c2_x = c2_A;
  c2_b_x = c2_x;
  c2_y = c2_b_x / 2.0;
  c2_c_x = c2_y;
  c2_d_x = c2_c_x;
  c2_d_x = muDoubleScalarCos(c2_d_x);
  c2_b_A = c2_pitch;
  c2_e_x = c2_b_A;
  c2_f_x = c2_e_x;
  c2_b_y = c2_f_x / 2.0;
  c2_g_x = c2_b_y;
  c2_h_x = c2_g_x;
  c2_h_x = muDoubleScalarCos(c2_h_x);
  c2_a = c2_d_x;
  c2_b = c2_h_x;
  c2_c_y = c2_a * c2_b;
  c2_c_A = c2_yaw;
  c2_i_x = c2_c_A;
  c2_j_x = c2_i_x;
  c2_d_y = c2_j_x / 2.0;
  c2_k_x = c2_d_y;
  c2_l_x = c2_k_x;
  c2_l_x = muDoubleScalarCos(c2_l_x);
  c2_b_a = c2_c_y;
  c2_b_b = c2_l_x;
  c2_e_y = c2_b_a * c2_b_b;
  c2_d_A = c2_roll;
  c2_m_x = c2_d_A;
  c2_n_x = c2_m_x;
  c2_f_y = c2_n_x / 2.0;
  c2_o_x = c2_f_y;
  c2_p_x = c2_o_x;
  c2_p_x = muDoubleScalarSin(c2_p_x);
  c2_e_A = c2_pitch;
  c2_q_x = c2_e_A;
  c2_r_x = c2_q_x;
  c2_g_y = c2_r_x / 2.0;
  c2_s_x = c2_g_y;
  c2_t_x = c2_s_x;
  c2_t_x = muDoubleScalarSin(c2_t_x);
  c2_c_a = c2_p_x;
  c2_c_b = c2_t_x;
  c2_h_y = c2_c_a * c2_c_b;
  c2_f_A = c2_yaw;
  c2_u_x = c2_f_A;
  c2_v_x = c2_u_x;
  c2_i_y = c2_v_x / 2.0;
  c2_w_x = c2_i_y;
  c2_x_x = c2_w_x;
  c2_x_x = muDoubleScalarSin(c2_x_x);
  c2_d_a = c2_h_y;
  c2_d_b = c2_x_x;
  c2_j_y = c2_d_a * c2_d_b;
  c2_quat[3] = c2_e_y + c2_j_y;
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 14);
  c2_g_A = c2_roll;
  c2_y_x = c2_g_A;
  c2_ab_x = c2_y_x;
  c2_k_y = c2_ab_x / 2.0;
  c2_bb_x = c2_k_y;
  c2_cb_x = c2_bb_x;
  c2_cb_x = muDoubleScalarSin(c2_cb_x);
  c2_h_A = c2_pitch;
  c2_db_x = c2_h_A;
  c2_eb_x = c2_db_x;
  c2_l_y = c2_eb_x / 2.0;
  c2_fb_x = c2_l_y;
  c2_gb_x = c2_fb_x;
  c2_gb_x = muDoubleScalarCos(c2_gb_x);
  c2_e_a = c2_cb_x;
  c2_e_b = c2_gb_x;
  c2_m_y = c2_e_a * c2_e_b;
  c2_i_A = c2_yaw;
  c2_hb_x = c2_i_A;
  c2_ib_x = c2_hb_x;
  c2_n_y = c2_ib_x / 2.0;
  c2_jb_x = c2_n_y;
  c2_kb_x = c2_jb_x;
  c2_kb_x = muDoubleScalarCos(c2_kb_x);
  c2_f_a = c2_m_y;
  c2_f_b = c2_kb_x;
  c2_o_y = c2_f_a * c2_f_b;
  c2_j_A = c2_roll;
  c2_lb_x = c2_j_A;
  c2_mb_x = c2_lb_x;
  c2_p_y = c2_mb_x / 2.0;
  c2_nb_x = c2_p_y;
  c2_ob_x = c2_nb_x;
  c2_ob_x = muDoubleScalarCos(c2_ob_x);
  c2_k_A = c2_pitch;
  c2_pb_x = c2_k_A;
  c2_qb_x = c2_pb_x;
  c2_q_y = c2_qb_x / 2.0;
  c2_rb_x = c2_q_y;
  c2_sb_x = c2_rb_x;
  c2_sb_x = muDoubleScalarSin(c2_sb_x);
  c2_g_a = c2_ob_x;
  c2_g_b = c2_sb_x;
  c2_r_y = c2_g_a * c2_g_b;
  c2_l_A = c2_yaw;
  c2_tb_x = c2_l_A;
  c2_ub_x = c2_tb_x;
  c2_s_y = c2_ub_x / 2.0;
  c2_vb_x = c2_s_y;
  c2_wb_x = c2_vb_x;
  c2_wb_x = muDoubleScalarSin(c2_wb_x);
  c2_h_a = c2_r_y;
  c2_h_b = c2_wb_x;
  c2_t_y = c2_h_a * c2_h_b;
  c2_quat[0] = c2_o_y - c2_t_y;
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 15);
  c2_m_A = c2_roll;
  c2_xb_x = c2_m_A;
  c2_yb_x = c2_xb_x;
  c2_u_y = c2_yb_x / 2.0;
  c2_ac_x = c2_u_y;
  c2_bc_x = c2_ac_x;
  c2_bc_x = muDoubleScalarCos(c2_bc_x);
  c2_n_A = c2_pitch;
  c2_cc_x = c2_n_A;
  c2_dc_x = c2_cc_x;
  c2_v_y = c2_dc_x / 2.0;
  c2_ec_x = c2_v_y;
  c2_fc_x = c2_ec_x;
  c2_fc_x = muDoubleScalarSin(c2_fc_x);
  c2_i_a = c2_bc_x;
  c2_i_b = c2_fc_x;
  c2_w_y = c2_i_a * c2_i_b;
  c2_o_A = c2_yaw;
  c2_gc_x = c2_o_A;
  c2_hc_x = c2_gc_x;
  c2_x_y = c2_hc_x / 2.0;
  c2_ic_x = c2_x_y;
  c2_jc_x = c2_ic_x;
  c2_jc_x = muDoubleScalarCos(c2_jc_x);
  c2_j_a = c2_w_y;
  c2_j_b = c2_jc_x;
  c2_y_y = c2_j_a * c2_j_b;
  c2_p_A = c2_roll;
  c2_kc_x = c2_p_A;
  c2_lc_x = c2_kc_x;
  c2_ab_y = c2_lc_x / 2.0;
  c2_mc_x = c2_ab_y;
  c2_nc_x = c2_mc_x;
  c2_nc_x = muDoubleScalarSin(c2_nc_x);
  c2_q_A = c2_pitch;
  c2_oc_x = c2_q_A;
  c2_pc_x = c2_oc_x;
  c2_bb_y = c2_pc_x / 2.0;
  c2_qc_x = c2_bb_y;
  c2_rc_x = c2_qc_x;
  c2_rc_x = muDoubleScalarCos(c2_rc_x);
  c2_k_a = c2_nc_x;
  c2_k_b = c2_rc_x;
  c2_cb_y = c2_k_a * c2_k_b;
  c2_r_A = c2_yaw;
  c2_sc_x = c2_r_A;
  c2_tc_x = c2_sc_x;
  c2_db_y = c2_tc_x / 2.0;
  c2_uc_x = c2_db_y;
  c2_vc_x = c2_uc_x;
  c2_vc_x = muDoubleScalarSin(c2_vc_x);
  c2_l_a = c2_cb_y;
  c2_l_b = c2_vc_x;
  c2_eb_y = c2_l_a * c2_l_b;
  c2_quat[1] = c2_y_y + c2_eb_y;
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 16);
  c2_s_A = c2_roll;
  c2_wc_x = c2_s_A;
  c2_xc_x = c2_wc_x;
  c2_fb_y = c2_xc_x / 2.0;
  c2_yc_x = c2_fb_y;
  c2_ad_x = c2_yc_x;
  c2_ad_x = muDoubleScalarCos(c2_ad_x);
  c2_t_A = c2_pitch;
  c2_bd_x = c2_t_A;
  c2_cd_x = c2_bd_x;
  c2_gb_y = c2_cd_x / 2.0;
  c2_dd_x = c2_gb_y;
  c2_ed_x = c2_dd_x;
  c2_ed_x = muDoubleScalarCos(c2_ed_x);
  c2_m_a = c2_ad_x;
  c2_m_b = c2_ed_x;
  c2_hb_y = c2_m_a * c2_m_b;
  c2_u_A = c2_yaw;
  c2_fd_x = c2_u_A;
  c2_gd_x = c2_fd_x;
  c2_ib_y = c2_gd_x / 2.0;
  c2_hd_x = c2_ib_y;
  c2_id_x = c2_hd_x;
  c2_id_x = muDoubleScalarSin(c2_id_x);
  c2_n_a = c2_hb_y;
  c2_n_b = c2_id_x;
  c2_jb_y = c2_n_a * c2_n_b;
  c2_v_A = c2_roll;
  c2_jd_x = c2_v_A;
  c2_kd_x = c2_jd_x;
  c2_kb_y = c2_kd_x / 2.0;
  c2_ld_x = c2_kb_y;
  c2_md_x = c2_ld_x;
  c2_md_x = muDoubleScalarSin(c2_md_x);
  c2_w_A = c2_pitch;
  c2_nd_x = c2_w_A;
  c2_od_x = c2_nd_x;
  c2_lb_y = c2_od_x / 2.0;
  c2_pd_x = c2_lb_y;
  c2_qd_x = c2_pd_x;
  c2_qd_x = muDoubleScalarSin(c2_qd_x);
  c2_o_a = c2_md_x;
  c2_o_b = c2_qd_x;
  c2_mb_y = c2_o_a * c2_o_b;
  c2_x_A = c2_yaw;
  c2_rd_x = c2_x_A;
  c2_sd_x = c2_rd_x;
  c2_nb_y = c2_sd_x / 2.0;
  c2_td_x = c2_nb_y;
  c2_ud_x = c2_td_x;
  c2_ud_x = muDoubleScalarCos(c2_ud_x);
  c2_p_a = c2_mb_y;
  c2_p_b = c2_ud_x;
  c2_ob_y = c2_p_a * c2_p_b;
  c2_quat[2] = c2_jb_y - c2_ob_y;
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, -16);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_quat_mult(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  real_T c2_q_1[4], real_T c2_q_2[4], real_T c2_q_o[4])
{
  uint32_T c2_debug_family_var_map[6];
  real_T c2_W[16];
  real_T c2_b_q_o[4];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i37;
  real_T c2_a[16];
  int32_T c2_i38;
  real_T c2_b[4];
  int32_T c2_i39;
  int32_T c2_i40;
  int32_T c2_i41;
  real_T c2_C[4];
  int32_T c2_i42;
  int32_T c2_i43;
  int32_T c2_i44;
  int32_T c2_i45;
  int32_T c2_i46;
  int32_T c2_i47;
  int32_T c2_i48;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 7U, c2_d_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_W, 0U, c2_f_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_q_o, MAX_uint32_T,
    c2_e_sf_marshallOut, c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_1, 4U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_2, 5U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_o, MAX_uint32_T, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 4);
  c2_W[0] = c2_q_2[3];
  c2_W[4] = c2_q_2[2];
  c2_W[8] = -c2_q_2[1];
  c2_W[12] = c2_q_2[0];
  c2_W[1] = -c2_q_2[2];
  c2_W[5] = c2_q_2[3];
  c2_W[9] = c2_q_2[0];
  c2_W[13] = c2_q_2[1];
  c2_W[2] = c2_q_2[1];
  c2_W[6] = -c2_q_2[0];
  c2_W[10] = c2_q_2[3];
  c2_W[14] = c2_q_2[2];
  c2_W[3] = -c2_q_2[0];
  c2_W[7] = -c2_q_2[1];
  c2_W[11] = -c2_q_2[2];
  c2_W[15] = c2_q_2[3];
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 9);
  for (c2_i37 = 0; c2_i37 < 16; c2_i37++) {
    c2_a[c2_i37] = c2_W[c2_i37];
  }

  for (c2_i38 = 0; c2_i38 < 4; c2_i38++) {
    c2_b[c2_i38] = c2_q_1[c2_i38];
  }

  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i39 = 0; c2_i39 < 4; c2_i39++) {
    c2_b_q_o[c2_i39] = 0.0;
  }

  for (c2_i40 = 0; c2_i40 < 4; c2_i40++) {
    c2_b_q_o[c2_i40] = 0.0;
  }

  for (c2_i41 = 0; c2_i41 < 4; c2_i41++) {
    c2_C[c2_i41] = c2_b_q_o[c2_i41];
  }

  for (c2_i42 = 0; c2_i42 < 4; c2_i42++) {
    c2_b_q_o[c2_i42] = c2_C[c2_i42];
  }

  for (c2_i43 = 0; c2_i43 < 4; c2_i43++) {
    c2_C[c2_i43] = c2_b_q_o[c2_i43];
  }

  for (c2_i44 = 0; c2_i44 < 4; c2_i44++) {
    c2_b_q_o[c2_i44] = c2_C[c2_i44];
  }

  for (c2_i45 = 0; c2_i45 < 4; c2_i45++) {
    c2_b_q_o[c2_i45] = 0.0;
    c2_i46 = 0;
    for (c2_i47 = 0; c2_i47 < 4; c2_i47++) {
      c2_b_q_o[c2_i45] += c2_a[c2_i46 + c2_i45] * c2_b[c2_i47];
      c2_i46 += 4;
    }
  }

  _SFD_SYMBOL_SWITCH(1U, 1U);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 10);
  for (c2_i48 = 0; c2_i48 < 4; c2_i48++) {
    c2_q_o[c2_i48] = c2_b_q_o[c2_i48];
  }

  _SFD_SYMBOL_SWITCH(1U, 6U);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, -10);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 0U, sf_debug_get_script_id(
    "/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/angle_to_quat.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 1U, sf_debug_get_script_id(
    "/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/quat_conjugate.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 2U, sf_debug_get_script_id(
    "/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/quat_mult.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 3U, sf_debug_get_script_id(
    "/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/sign_l.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 4U, sf_debug_get_script_id(
    "/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/get_z_from_quat.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 5U, sf_debug_get_script_id(
    "/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/quat_inv_multiply.m"));
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_phi_up, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_phi_up), &c2_thisId);
  sf_mex_destroy(&c2_phi_up);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_phi_up;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_phi_up = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_phi_up), &c2_thisId);
  sf_mex_destroy(&c2_phi_up);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i49;
  real_T c2_b_inData[3];
  int32_T c2_i50;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i49 = 0; c2_i49 < 3; c2_i49++) {
    c2_b_inData[c2_i49] = (*(real_T (*)[3])c2_inData)[c2_i49];
  }

  for (c2_i50 = 0; c2_i50 < 3; c2_i50++) {
    c2_u[c2_i50] = c2_b_inData[c2_i50];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_Torques, const char_T *c2_identifier, real_T
  c2_y[3])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Torques), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Torques);
}

static void c2_d_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3])
{
  real_T c2_dv8[3];
  int32_T c2_i51;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv8, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i51 = 0; c2_i51 < 3; c2_i51++) {
    c2_y[c2_i51] = c2_dv8[c2_i51];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Torques;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i52;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_Torques = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Torques), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Torques);
  for (c2_i52 = 0; c2_i52 < 3; c2_i52++) {
    (*(real_T (*)[3])c2_outData)[c2_i52] = c2_y[c2_i52];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i53;
  real_T c2_b_inData[3];
  int32_T c2_i54;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i53 = 0; c2_i53 < 3; c2_i53++) {
    c2_b_inData[c2_i53] = (*(real_T (*)[3])c2_inData)[c2_i53];
  }

  for (c2_i54 = 0; c2_i54 < 3; c2_i54++) {
    c2_u[c2_i54] = c2_b_inData[c2_i54];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_e_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3])
{
  real_T c2_dv9[3];
  int32_T c2_i55;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv9, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c2_i55 = 0; c2_i55 < 3; c2_i55++) {
    c2_y[c2_i55] = c2_dv9[c2_i55];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Torque_field;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i56;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_Torque_field = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Torque_field), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_Torque_field);
  for (c2_i56 = 0; c2_i56 < 3; c2_i56++) {
    (*(real_T (*)[3])c2_outData)[c2_i56] = c2_y[c2_i56];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i57;
  real_T c2_b_inData[4];
  int32_T c2_i58;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i57 = 0; c2_i57 < 4; c2_i57++) {
    c2_b_inData[c2_i57] = (*(real_T (*)[4])c2_inData)[c2_i57];
  }

  for (c2_i58 = 0; c2_i58 < 4; c2_i58++) {
    c2_u[c2_i58] = c2_b_inData[c2_i58];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_f_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv10[4];
  int32_T c2_i59;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv10, 1, 0, 0U, 1, 0U, 2, 1, 4);
  for (c2_i59 = 0; c2_i59 < 4; c2_i59++) {
    c2_y[c2_i59] = c2_dv10[c2_i59];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_q_xy;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i60;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_q_xy = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_q_xy), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_q_xy);
  for (c2_i60 = 0; c2_i60 < 4; c2_i60++) {
    (*(real_T (*)[4])c2_outData)[c2_i60] = c2_y[c2_i60];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i61;
  real_T c2_b_inData[4];
  int32_T c2_i62;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i61 = 0; c2_i61 < 4; c2_i61++) {
    c2_b_inData[c2_i61] = (*(real_T (*)[4])c2_inData)[c2_i61];
  }

  for (c2_i62 = 0; c2_i62 < 4; c2_i62++) {
    c2_u[c2_i62] = c2_b_inData[c2_i62];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_g_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv11[4];
  int32_T c2_i63;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv11, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i63 = 0; c2_i63 < 4; c2_i63++) {
    c2_y[c2_i63] = c2_dv11[c2_i63];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_q_o;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i64;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_q_o = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_q_o), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_q_o);
  for (c2_i64 = 0; c2_i64 < 4; c2_i64++) {
    (*(real_T (*)[4])c2_outData)[c2_i64] = c2_y[c2_i64];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i65;
  int32_T c2_i66;
  int32_T c2_i67;
  real_T c2_b_inData[16];
  int32_T c2_i68;
  int32_T c2_i69;
  int32_T c2_i70;
  real_T c2_u[16];
  const mxArray *c2_y = NULL;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i65 = 0;
  for (c2_i66 = 0; c2_i66 < 4; c2_i66++) {
    for (c2_i67 = 0; c2_i67 < 4; c2_i67++) {
      c2_b_inData[c2_i67 + c2_i65] = (*(real_T (*)[16])c2_inData)[c2_i67 +
        c2_i65];
    }

    c2_i65 += 4;
  }

  c2_i68 = 0;
  for (c2_i69 = 0; c2_i69 < 4; c2_i69++) {
    for (c2_i70 = 0; c2_i70 < 4; c2_i70++) {
      c2_u[c2_i70 + c2_i68] = c2_b_inData[c2_i70 + c2_i68];
    }

    c2_i68 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_h_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16])
{
  real_T c2_dv12[16];
  int32_T c2_i71;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv12, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c2_i71 = 0; c2_i71 < 16; c2_i71++) {
    c2_y[c2_i71] = c2_dv12[c2_i71];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_W;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[16];
  int32_T c2_i72;
  int32_T c2_i73;
  int32_T c2_i74;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_W = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_W), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_W);
  c2_i72 = 0;
  for (c2_i73 = 0; c2_i73 < 4; c2_i73++) {
    for (c2_i74 = 0; c2_i74 < 4; c2_i74++) {
      (*(real_T (*)[16])c2_outData)[c2_i74 + c2_i72] = c2_y[c2_i74 + c2_i72];
    }

    c2_i72 += 4;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_Thrust_Direction_quad_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[148];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i75;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  c2_b_info_helper(c2_info);
  c2_c_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 148), FALSE);
  for (c2_i75 = 0; c2_i75 < 148; c2_i75++) {
    c2_r0 = &c2_info[c2_i75];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i75);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i75);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i75);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i75);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i75);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i75);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i75);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i75);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[148])
{
  c2_info[0].context = "";
  c2_info[0].name = "angle_to_quat";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/angle_to_quat.m";
  c2_info[0].fileTimeLo = 1390815489U;
  c2_info[0].fileTimeHi = 0U;
  c2_info[0].mFileTimeLo = 0U;
  c2_info[0].mFileTimeHi = 0U;
  c2_info[1].context =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/angle_to_quat.m";
  c2_info[1].name = "mrdivide";
  c2_info[1].dominantType = "double";
  c2_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[1].fileTimeLo = 1357951548U;
  c2_info[1].fileTimeHi = 0U;
  c2_info[1].mFileTimeLo = 1319729966U;
  c2_info[1].mFileTimeHi = 0U;
  c2_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[2].name = "rdivide";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[2].fileTimeLo = 1346510388U;
  c2_info[2].fileTimeHi = 0U;
  c2_info[2].mFileTimeLo = 0U;
  c2_info[2].mFileTimeHi = 0U;
  c2_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[3].name = "eml_scalexp_compatible";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c2_info[3].fileTimeLo = 1286818796U;
  c2_info[3].fileTimeHi = 0U;
  c2_info[3].mFileTimeLo = 0U;
  c2_info[3].mFileTimeHi = 0U;
  c2_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[4].name = "eml_div";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[4].fileTimeLo = 1313347810U;
  c2_info[4].fileTimeHi = 0U;
  c2_info[4].mFileTimeLo = 0U;
  c2_info[4].mFileTimeHi = 0U;
  c2_info[5].context =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/angle_to_quat.m";
  c2_info[5].name = "cos";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[5].fileTimeLo = 1343830372U;
  c2_info[5].fileTimeHi = 0U;
  c2_info[5].mFileTimeLo = 0U;
  c2_info[5].mFileTimeHi = 0U;
  c2_info[6].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[6].name = "eml_scalar_cos";
  c2_info[6].dominantType = "double";
  c2_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c2_info[6].fileTimeLo = 1286818722U;
  c2_info[6].fileTimeHi = 0U;
  c2_info[6].mFileTimeLo = 0U;
  c2_info[6].mFileTimeHi = 0U;
  c2_info[7].context =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/angle_to_quat.m";
  c2_info[7].name = "mtimes";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[7].fileTimeLo = 1289519692U;
  c2_info[7].fileTimeHi = 0U;
  c2_info[7].mFileTimeLo = 0U;
  c2_info[7].mFileTimeHi = 0U;
  c2_info[8].context =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/angle_to_quat.m";
  c2_info[8].name = "sin";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[8].fileTimeLo = 1343830386U;
  c2_info[8].fileTimeHi = 0U;
  c2_info[8].mFileTimeLo = 0U;
  c2_info[8].mFileTimeHi = 0U;
  c2_info[9].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[9].name = "eml_scalar_sin";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c2_info[9].fileTimeLo = 1286818736U;
  c2_info[9].fileTimeHi = 0U;
  c2_info[9].mFileTimeLo = 0U;
  c2_info[9].mFileTimeHi = 0U;
  c2_info[10].context = "";
  c2_info[10].name = "quat_conjugate";
  c2_info[10].dominantType = "double";
  c2_info[10].resolved =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/quat_conjugate.m";
  c2_info[10].fileTimeLo = 1390815308U;
  c2_info[10].fileTimeHi = 0U;
  c2_info[10].mFileTimeLo = 0U;
  c2_info[10].mFileTimeHi = 0U;
  c2_info[11].context = "";
  c2_info[11].name = "quat_mult";
  c2_info[11].dominantType = "double";
  c2_info[11].resolved =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/quat_mult.m";
  c2_info[11].fileTimeLo = 1390836187U;
  c2_info[11].fileTimeHi = 0U;
  c2_info[11].mFileTimeLo = 0U;
  c2_info[11].mFileTimeHi = 0U;
  c2_info[12].context =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/quat_mult.m";
  c2_info[12].name = "mtimes";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[12].fileTimeLo = 1289519692U;
  c2_info[12].fileTimeHi = 0U;
  c2_info[12].mFileTimeLo = 0U;
  c2_info[12].mFileTimeHi = 0U;
  c2_info[13].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[13].name = "eml_index_class";
  c2_info[13].dominantType = "";
  c2_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[13].fileTimeLo = 1323170578U;
  c2_info[13].fileTimeHi = 0U;
  c2_info[13].mFileTimeLo = 0U;
  c2_info[13].mFileTimeHi = 0U;
  c2_info[14].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[14].name = "eml_scalar_eg";
  c2_info[14].dominantType = "double";
  c2_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[14].fileTimeLo = 1286818796U;
  c2_info[14].fileTimeHi = 0U;
  c2_info[14].mFileTimeLo = 0U;
  c2_info[14].mFileTimeHi = 0U;
  c2_info[15].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[15].name = "eml_xgemm";
  c2_info[15].dominantType = "char";
  c2_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[15].fileTimeLo = 1299076772U;
  c2_info[15].fileTimeHi = 0U;
  c2_info[15].mFileTimeLo = 0U;
  c2_info[15].mFileTimeHi = 0U;
  c2_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[16].name = "eml_blas_inline";
  c2_info[16].dominantType = "";
  c2_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[16].fileTimeLo = 1299076768U;
  c2_info[16].fileTimeHi = 0U;
  c2_info[16].mFileTimeLo = 0U;
  c2_info[16].mFileTimeHi = 0U;
  c2_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c2_info[17].name = "mtimes";
  c2_info[17].dominantType = "double";
  c2_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[17].fileTimeLo = 1289519692U;
  c2_info[17].fileTimeHi = 0U;
  c2_info[17].mFileTimeLo = 0U;
  c2_info[17].mFileTimeHi = 0U;
  c2_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[18].name = "eml_index_class";
  c2_info[18].dominantType = "";
  c2_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[18].fileTimeLo = 1323170578U;
  c2_info[18].fileTimeHi = 0U;
  c2_info[18].mFileTimeLo = 0U;
  c2_info[18].mFileTimeHi = 0U;
  c2_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[19].name = "eml_scalar_eg";
  c2_info[19].dominantType = "double";
  c2_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[19].fileTimeLo = 1286818796U;
  c2_info[19].fileTimeHi = 0U;
  c2_info[19].mFileTimeLo = 0U;
  c2_info[19].mFileTimeHi = 0U;
  c2_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[20].name = "eml_refblas_xgemm";
  c2_info[20].dominantType = "char";
  c2_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[20].fileTimeLo = 1299076774U;
  c2_info[20].fileTimeHi = 0U;
  c2_info[20].mFileTimeLo = 0U;
  c2_info[20].mFileTimeHi = 0U;
  c2_info[21].context = "";
  c2_info[21].name = "sign_l";
  c2_info[21].dominantType = "double";
  c2_info[21].resolved =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/sign_l.m";
  c2_info[21].fileTimeLo = 1390811159U;
  c2_info[21].fileTimeHi = 0U;
  c2_info[21].mFileTimeLo = 0U;
  c2_info[21].mFileTimeHi = 0U;
  c2_info[22].context = "";
  c2_info[22].name = "get_z_from_quat";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/get_z_from_quat.m";
  c2_info[22].fileTimeLo = 1390815805U;
  c2_info[22].fileTimeHi = 0U;
  c2_info[22].mFileTimeLo = 0U;
  c2_info[22].mFileTimeHi = 0U;
  c2_info[23].context =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/get_z_from_quat.m";
  c2_info[23].name = "mpower";
  c2_info[23].dominantType = "double";
  c2_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[23].fileTimeLo = 1286818842U;
  c2_info[23].fileTimeHi = 0U;
  c2_info[23].mFileTimeLo = 0U;
  c2_info[23].mFileTimeHi = 0U;
  c2_info[24].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[24].name = "power";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[24].fileTimeLo = 1348191930U;
  c2_info[24].fileTimeHi = 0U;
  c2_info[24].mFileTimeLo = 0U;
  c2_info[24].mFileTimeHi = 0U;
  c2_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[25].name = "eml_scalar_eg";
  c2_info[25].dominantType = "double";
  c2_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[25].fileTimeLo = 1286818796U;
  c2_info[25].fileTimeHi = 0U;
  c2_info[25].mFileTimeLo = 0U;
  c2_info[25].mFileTimeHi = 0U;
  c2_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[26].name = "eml_scalexp_alloc";
  c2_info[26].dominantType = "double";
  c2_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[26].fileTimeLo = 1352424860U;
  c2_info[26].fileTimeHi = 0U;
  c2_info[26].mFileTimeLo = 0U;
  c2_info[26].mFileTimeHi = 0U;
  c2_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[27].name = "floor";
  c2_info[27].dominantType = "double";
  c2_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[27].fileTimeLo = 1343830380U;
  c2_info[27].fileTimeHi = 0U;
  c2_info[27].mFileTimeLo = 0U;
  c2_info[27].mFileTimeHi = 0U;
  c2_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[28].name = "eml_scalar_floor";
  c2_info[28].dominantType = "double";
  c2_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[28].fileTimeLo = 1286818726U;
  c2_info[28].fileTimeHi = 0U;
  c2_info[28].mFileTimeLo = 0U;
  c2_info[28].mFileTimeHi = 0U;
  c2_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c2_info[29].name = "eml_scalar_eg";
  c2_info[29].dominantType = "double";
  c2_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[29].fileTimeLo = 1286818796U;
  c2_info[29].fileTimeHi = 0U;
  c2_info[29].mFileTimeLo = 0U;
  c2_info[29].mFileTimeHi = 0U;
  c2_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c2_info[30].name = "mtimes";
  c2_info[30].dominantType = "double";
  c2_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[30].fileTimeLo = 1289519692U;
  c2_info[30].fileTimeHi = 0U;
  c2_info[30].mFileTimeLo = 0U;
  c2_info[30].mFileTimeHi = 0U;
  c2_info[31].context =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/get_z_from_quat.m";
  c2_info[31].name = "sqrt";
  c2_info[31].dominantType = "double";
  c2_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[31].fileTimeLo = 1343830386U;
  c2_info[31].fileTimeHi = 0U;
  c2_info[31].mFileTimeLo = 0U;
  c2_info[31].mFileTimeHi = 0U;
  c2_info[32].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[32].name = "eml_error";
  c2_info[32].dominantType = "char";
  c2_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[32].fileTimeLo = 1343830358U;
  c2_info[32].fileTimeHi = 0U;
  c2_info[32].mFileTimeLo = 0U;
  c2_info[32].mFileTimeHi = 0U;
  c2_info[33].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[33].name = "eml_scalar_sqrt";
  c2_info[33].dominantType = "double";
  c2_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c2_info[33].fileTimeLo = 1286818738U;
  c2_info[33].fileTimeHi = 0U;
  c2_info[33].mFileTimeLo = 0U;
  c2_info[33].mFileTimeHi = 0U;
  c2_info[34].context =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/get_z_from_quat.m";
  c2_info[34].name = "mrdivide";
  c2_info[34].dominantType = "double";
  c2_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[34].fileTimeLo = 1357951548U;
  c2_info[34].fileTimeHi = 0U;
  c2_info[34].mFileTimeLo = 1319729966U;
  c2_info[34].mFileTimeHi = 0U;
  c2_info[35].context = "";
  c2_info[35].name = "quat_inv_multiply";
  c2_info[35].dominantType = "double";
  c2_info[35].resolved =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/quat_inv_multiply.m";
  c2_info[35].fileTimeLo = 1390816063U;
  c2_info[35].fileTimeHi = 0U;
  c2_info[35].mFileTimeLo = 0U;
  c2_info[35].mFileTimeHi = 0U;
  c2_info[36].context =
    "[E]/home/diogo/Documents/Dropbox/SCR/Thesis/MATLAB/Quad_Model/quat_inv_multiply.m";
  c2_info[36].name = "mldivide";
  c2_info[36].dominantType = "double";
  c2_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[36].fileTimeLo = 1357951548U;
  c2_info[36].fileTimeHi = 0U;
  c2_info[36].mFileTimeLo = 1319729966U;
  c2_info[36].mFileTimeHi = 0U;
  c2_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[37].name = "eml_lusolve";
  c2_info[37].dominantType = "double";
  c2_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c2_info[37].fileTimeLo = 1309451196U;
  c2_info[37].fileTimeHi = 0U;
  c2_info[37].mFileTimeLo = 0U;
  c2_info[37].mFileTimeHi = 0U;
  c2_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c2_info[38].name = "eml_index_class";
  c2_info[38].dominantType = "";
  c2_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[38].fileTimeLo = 1323170578U;
  c2_info[38].fileTimeHi = 0U;
  c2_info[38].mFileTimeLo = 0U;
  c2_info[38].mFileTimeHi = 0U;
  c2_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[39].name = "eml_index_class";
  c2_info[39].dominantType = "";
  c2_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[39].fileTimeLo = 1323170578U;
  c2_info[39].fileTimeHi = 0U;
  c2_info[39].mFileTimeLo = 0U;
  c2_info[39].mFileTimeHi = 0U;
  c2_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[40].name = "eml_xgetrf";
  c2_info[40].dominantType = "double";
  c2_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c2_info[40].fileTimeLo = 1286818806U;
  c2_info[40].fileTimeHi = 0U;
  c2_info[40].mFileTimeLo = 0U;
  c2_info[40].mFileTimeHi = 0U;
  c2_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c2_info[41].name = "eml_lapack_xgetrf";
  c2_info[41].dominantType = "double";
  c2_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c2_info[41].fileTimeLo = 1286818810U;
  c2_info[41].fileTimeHi = 0U;
  c2_info[41].mFileTimeLo = 0U;
  c2_info[41].mFileTimeHi = 0U;
  c2_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c2_info[42].name = "eml_matlab_zgetrf";
  c2_info[42].dominantType = "double";
  c2_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[42].fileTimeLo = 1302688994U;
  c2_info[42].fileTimeHi = 0U;
  c2_info[42].mFileTimeLo = 0U;
  c2_info[42].mFileTimeHi = 0U;
  c2_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[43].name = "realmin";
  c2_info[43].dominantType = "char";
  c2_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[43].fileTimeLo = 1307651242U;
  c2_info[43].fileTimeHi = 0U;
  c2_info[43].mFileTimeLo = 0U;
  c2_info[43].mFileTimeHi = 0U;
  c2_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[44].name = "eml_realmin";
  c2_info[44].dominantType = "char";
  c2_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[44].fileTimeLo = 1307651244U;
  c2_info[44].fileTimeHi = 0U;
  c2_info[44].mFileTimeLo = 0U;
  c2_info[44].mFileTimeHi = 0U;
  c2_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[45].name = "eml_float_model";
  c2_info[45].dominantType = "char";
  c2_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[45].fileTimeLo = 1326727996U;
  c2_info[45].fileTimeHi = 0U;
  c2_info[45].mFileTimeLo = 0U;
  c2_info[45].mFileTimeHi = 0U;
  c2_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[46].name = "eps";
  c2_info[46].dominantType = "char";
  c2_info[46].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[46].fileTimeLo = 1326727996U;
  c2_info[46].fileTimeHi = 0U;
  c2_info[46].mFileTimeLo = 0U;
  c2_info[46].mFileTimeHi = 0U;
  c2_info[47].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[47].name = "eml_is_float_class";
  c2_info[47].dominantType = "char";
  c2_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[47].fileTimeLo = 1286818782U;
  c2_info[47].fileTimeHi = 0U;
  c2_info[47].mFileTimeLo = 0U;
  c2_info[47].mFileTimeHi = 0U;
  c2_info[48].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[48].name = "eml_eps";
  c2_info[48].dominantType = "char";
  c2_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[48].fileTimeLo = 1326727996U;
  c2_info[48].fileTimeHi = 0U;
  c2_info[48].mFileTimeLo = 0U;
  c2_info[48].mFileTimeHi = 0U;
  c2_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[49].name = "eml_float_model";
  c2_info[49].dominantType = "char";
  c2_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[49].fileTimeLo = 1326727996U;
  c2_info[49].fileTimeHi = 0U;
  c2_info[49].mFileTimeLo = 0U;
  c2_info[49].mFileTimeHi = 0U;
  c2_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[50].name = "min";
  c2_info[50].dominantType = "coder.internal.indexInt";
  c2_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[50].fileTimeLo = 1311255318U;
  c2_info[50].fileTimeHi = 0U;
  c2_info[50].mFileTimeLo = 0U;
  c2_info[50].mFileTimeHi = 0U;
  c2_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[51].name = "eml_min_or_max";
  c2_info[51].dominantType = "char";
  c2_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[51].fileTimeLo = 1334071490U;
  c2_info[51].fileTimeHi = 0U;
  c2_info[51].mFileTimeLo = 0U;
  c2_info[51].mFileTimeHi = 0U;
  c2_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[52].name = "eml_scalar_eg";
  c2_info[52].dominantType = "coder.internal.indexInt";
  c2_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[52].fileTimeLo = 1286818796U;
  c2_info[52].fileTimeHi = 0U;
  c2_info[52].mFileTimeLo = 0U;
  c2_info[52].mFileTimeHi = 0U;
  c2_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[53].name = "eml_scalexp_alloc";
  c2_info[53].dominantType = "coder.internal.indexInt";
  c2_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[53].fileTimeLo = 1352424860U;
  c2_info[53].fileTimeHi = 0U;
  c2_info[53].mFileTimeLo = 0U;
  c2_info[53].mFileTimeHi = 0U;
  c2_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[54].name = "eml_index_class";
  c2_info[54].dominantType = "";
  c2_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[54].fileTimeLo = 1323170578U;
  c2_info[54].fileTimeHi = 0U;
  c2_info[54].mFileTimeLo = 0U;
  c2_info[54].mFileTimeHi = 0U;
  c2_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[55].name = "eml_scalar_eg";
  c2_info[55].dominantType = "coder.internal.indexInt";
  c2_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[55].fileTimeLo = 1286818796U;
  c2_info[55].fileTimeHi = 0U;
  c2_info[55].mFileTimeLo = 0U;
  c2_info[55].mFileTimeHi = 0U;
  c2_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[56].name = "colon";
  c2_info[56].dominantType = "double";
  c2_info[56].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[56].fileTimeLo = 1348191928U;
  c2_info[56].fileTimeHi = 0U;
  c2_info[56].mFileTimeLo = 0U;
  c2_info[56].mFileTimeHi = 0U;
  c2_info[57].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[57].name = "colon";
  c2_info[57].dominantType = "double";
  c2_info[57].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[57].fileTimeLo = 1348191928U;
  c2_info[57].fileTimeHi = 0U;
  c2_info[57].mFileTimeLo = 0U;
  c2_info[57].mFileTimeHi = 0U;
  c2_info[58].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[58].name = "floor";
  c2_info[58].dominantType = "double";
  c2_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[58].fileTimeLo = 1343830380U;
  c2_info[58].fileTimeHi = 0U;
  c2_info[58].mFileTimeLo = 0U;
  c2_info[58].mFileTimeHi = 0U;
  c2_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[59].name = "intmin";
  c2_info[59].dominantType = "char";
  c2_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[59].fileTimeLo = 1311255318U;
  c2_info[59].fileTimeHi = 0U;
  c2_info[59].mFileTimeLo = 0U;
  c2_info[59].mFileTimeHi = 0U;
  c2_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[60].name = "intmax";
  c2_info[60].dominantType = "char";
  c2_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[60].fileTimeLo = 1311255316U;
  c2_info[60].fileTimeHi = 0U;
  c2_info[60].mFileTimeLo = 0U;
  c2_info[60].mFileTimeHi = 0U;
  c2_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[61].name = "intmin";
  c2_info[61].dominantType = "char";
  c2_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[61].fileTimeLo = 1311255318U;
  c2_info[61].fileTimeHi = 0U;
  c2_info[61].mFileTimeLo = 0U;
  c2_info[61].mFileTimeHi = 0U;
  c2_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[62].name = "intmax";
  c2_info[62].dominantType = "char";
  c2_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[62].fileTimeLo = 1311255316U;
  c2_info[62].fileTimeHi = 0U;
  c2_info[62].mFileTimeLo = 0U;
  c2_info[62].mFileTimeHi = 0U;
  c2_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[63].name = "eml_isa_uint";
  c2_info[63].dominantType = "coder.internal.indexInt";
  c2_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[63].fileTimeLo = 1286818784U;
  c2_info[63].fileTimeHi = 0U;
  c2_info[63].mFileTimeLo = 0U;
  c2_info[63].mFileTimeHi = 0U;
}

static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[148])
{
  c2_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[64].name = "eml_unsigned_class";
  c2_info[64].dominantType = "char";
  c2_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c2_info[64].fileTimeLo = 1323170580U;
  c2_info[64].fileTimeHi = 0U;
  c2_info[64].mFileTimeLo = 0U;
  c2_info[64].mFileTimeHi = 0U;
  c2_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c2_info[65].name = "eml_index_class";
  c2_info[65].dominantType = "";
  c2_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[65].fileTimeLo = 1323170578U;
  c2_info[65].fileTimeHi = 0U;
  c2_info[65].mFileTimeLo = 0U;
  c2_info[65].mFileTimeHi = 0U;
  c2_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[66].name = "eml_index_class";
  c2_info[66].dominantType = "";
  c2_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[66].fileTimeLo = 1323170578U;
  c2_info[66].fileTimeHi = 0U;
  c2_info[66].mFileTimeLo = 0U;
  c2_info[66].mFileTimeHi = 0U;
  c2_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[67].name = "intmax";
  c2_info[67].dominantType = "char";
  c2_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[67].fileTimeLo = 1311255316U;
  c2_info[67].fileTimeHi = 0U;
  c2_info[67].mFileTimeLo = 0U;
  c2_info[67].mFileTimeHi = 0U;
  c2_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[68].name = "eml_isa_uint";
  c2_info[68].dominantType = "coder.internal.indexInt";
  c2_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[68].fileTimeLo = 1286818784U;
  c2_info[68].fileTimeHi = 0U;
  c2_info[68].mFileTimeLo = 0U;
  c2_info[68].mFileTimeHi = 0U;
  c2_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[69].name = "eml_index_plus";
  c2_info[69].dominantType = "double";
  c2_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[69].fileTimeLo = 1286818778U;
  c2_info[69].fileTimeHi = 0U;
  c2_info[69].mFileTimeLo = 0U;
  c2_info[69].mFileTimeHi = 0U;
  c2_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[70].name = "eml_index_class";
  c2_info[70].dominantType = "";
  c2_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[70].fileTimeLo = 1323170578U;
  c2_info[70].fileTimeHi = 0U;
  c2_info[70].mFileTimeLo = 0U;
  c2_info[70].mFileTimeHi = 0U;
  c2_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c2_info[71].name = "eml_int_forloop_overflow_check";
  c2_info[71].dominantType = "";
  c2_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[71].fileTimeLo = 1346510340U;
  c2_info[71].fileTimeHi = 0U;
  c2_info[71].mFileTimeLo = 0U;
  c2_info[71].mFileTimeHi = 0U;
  c2_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[72].name = "intmax";
  c2_info[72].dominantType = "char";
  c2_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[72].fileTimeLo = 1311255316U;
  c2_info[72].fileTimeHi = 0U;
  c2_info[72].mFileTimeLo = 0U;
  c2_info[72].mFileTimeHi = 0U;
  c2_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[73].name = "eml_index_class";
  c2_info[73].dominantType = "";
  c2_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[73].fileTimeLo = 1323170578U;
  c2_info[73].fileTimeHi = 0U;
  c2_info[73].mFileTimeLo = 0U;
  c2_info[73].mFileTimeHi = 0U;
  c2_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[74].name = "eml_index_plus";
  c2_info[74].dominantType = "double";
  c2_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[74].fileTimeLo = 1286818778U;
  c2_info[74].fileTimeHi = 0U;
  c2_info[74].mFileTimeLo = 0U;
  c2_info[74].mFileTimeHi = 0U;
  c2_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[75].name = "eml_int_forloop_overflow_check";
  c2_info[75].dominantType = "";
  c2_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[75].fileTimeLo = 1346510340U;
  c2_info[75].fileTimeHi = 0U;
  c2_info[75].mFileTimeLo = 0U;
  c2_info[75].mFileTimeHi = 0U;
  c2_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[76].name = "eml_index_minus";
  c2_info[76].dominantType = "double";
  c2_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[76].fileTimeLo = 1286818778U;
  c2_info[76].fileTimeHi = 0U;
  c2_info[76].mFileTimeLo = 0U;
  c2_info[76].mFileTimeHi = 0U;
  c2_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[77].name = "eml_index_class";
  c2_info[77].dominantType = "";
  c2_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[77].fileTimeLo = 1323170578U;
  c2_info[77].fileTimeHi = 0U;
  c2_info[77].mFileTimeLo = 0U;
  c2_info[77].mFileTimeHi = 0U;
  c2_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[78].name = "eml_index_minus";
  c2_info[78].dominantType = "coder.internal.indexInt";
  c2_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[78].fileTimeLo = 1286818778U;
  c2_info[78].fileTimeHi = 0U;
  c2_info[78].mFileTimeLo = 0U;
  c2_info[78].mFileTimeHi = 0U;
  c2_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[79].name = "eml_index_times";
  c2_info[79].dominantType = "coder.internal.indexInt";
  c2_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[79].fileTimeLo = 1286818780U;
  c2_info[79].fileTimeHi = 0U;
  c2_info[79].mFileTimeLo = 0U;
  c2_info[79].mFileTimeHi = 0U;
  c2_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[80].name = "eml_index_class";
  c2_info[80].dominantType = "";
  c2_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[80].fileTimeLo = 1323170578U;
  c2_info[80].fileTimeHi = 0U;
  c2_info[80].mFileTimeLo = 0U;
  c2_info[80].mFileTimeHi = 0U;
  c2_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[81].name = "eml_index_plus";
  c2_info[81].dominantType = "coder.internal.indexInt";
  c2_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[81].fileTimeLo = 1286818778U;
  c2_info[81].fileTimeHi = 0U;
  c2_info[81].mFileTimeLo = 0U;
  c2_info[81].mFileTimeHi = 0U;
  c2_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[82].name = "eml_ixamax";
  c2_info[82].dominantType = "double";
  c2_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[82].fileTimeLo = 1299076770U;
  c2_info[82].fileTimeHi = 0U;
  c2_info[82].mFileTimeLo = 0U;
  c2_info[82].mFileTimeHi = 0U;
  c2_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[83].name = "eml_blas_inline";
  c2_info[83].dominantType = "";
  c2_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[83].fileTimeLo = 1299076768U;
  c2_info[83].fileTimeHi = 0U;
  c2_info[83].mFileTimeLo = 0U;
  c2_info[83].mFileTimeHi = 0U;
  c2_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c2_info[84].name = "length";
  c2_info[84].dominantType = "double";
  c2_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[84].fileTimeLo = 1303146206U;
  c2_info[84].fileTimeHi = 0U;
  c2_info[84].mFileTimeLo = 0U;
  c2_info[84].mFileTimeHi = 0U;
  c2_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c2_info[85].name = "eml_index_class";
  c2_info[85].dominantType = "";
  c2_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[85].fileTimeLo = 1323170578U;
  c2_info[85].fileTimeHi = 0U;
  c2_info[85].mFileTimeLo = 0U;
  c2_info[85].mFileTimeHi = 0U;
  c2_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c2_info[86].name = "eml_index_class";
  c2_info[86].dominantType = "";
  c2_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[86].fileTimeLo = 1323170578U;
  c2_info[86].fileTimeHi = 0U;
  c2_info[86].mFileTimeLo = 0U;
  c2_info[86].mFileTimeHi = 0U;
  c2_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c2_info[87].name = "eml_refblas_ixamax";
  c2_info[87].dominantType = "double";
  c2_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[87].fileTimeLo = 1299076770U;
  c2_info[87].fileTimeHi = 0U;
  c2_info[87].mFileTimeLo = 0U;
  c2_info[87].mFileTimeHi = 0U;
  c2_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[88].name = "eml_index_class";
  c2_info[88].dominantType = "";
  c2_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[88].fileTimeLo = 1323170578U;
  c2_info[88].fileTimeHi = 0U;
  c2_info[88].mFileTimeLo = 0U;
  c2_info[88].mFileTimeHi = 0U;
  c2_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[89].name = "eml_xcabs1";
  c2_info[89].dominantType = "double";
  c2_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[89].fileTimeLo = 1286818706U;
  c2_info[89].fileTimeHi = 0U;
  c2_info[89].mFileTimeLo = 0U;
  c2_info[89].mFileTimeHi = 0U;
  c2_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[90].name = "abs";
  c2_info[90].dominantType = "double";
  c2_info[90].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[90].fileTimeLo = 1343830366U;
  c2_info[90].fileTimeHi = 0U;
  c2_info[90].mFileTimeLo = 0U;
  c2_info[90].mFileTimeHi = 0U;
  c2_info[91].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[91].name = "eml_scalar_abs";
  c2_info[91].dominantType = "double";
  c2_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[91].fileTimeLo = 1286818712U;
  c2_info[91].fileTimeHi = 0U;
  c2_info[91].mFileTimeLo = 0U;
  c2_info[91].mFileTimeHi = 0U;
  c2_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[92].name = "eml_int_forloop_overflow_check";
  c2_info[92].dominantType = "";
  c2_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[92].fileTimeLo = 1346510340U;
  c2_info[92].fileTimeHi = 0U;
  c2_info[92].mFileTimeLo = 0U;
  c2_info[92].mFileTimeHi = 0U;
  c2_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[93].name = "eml_index_plus";
  c2_info[93].dominantType = "coder.internal.indexInt";
  c2_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[93].fileTimeLo = 1286818778U;
  c2_info[93].fileTimeHi = 0U;
  c2_info[93].mFileTimeLo = 0U;
  c2_info[93].mFileTimeHi = 0U;
  c2_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[94].name = "eml_xswap";
  c2_info[94].dominantType = "double";
  c2_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[94].fileTimeLo = 1299076778U;
  c2_info[94].fileTimeHi = 0U;
  c2_info[94].mFileTimeLo = 0U;
  c2_info[94].mFileTimeHi = 0U;
  c2_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[95].name = "eml_blas_inline";
  c2_info[95].dominantType = "";
  c2_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[95].fileTimeLo = 1299076768U;
  c2_info[95].fileTimeHi = 0U;
  c2_info[95].mFileTimeLo = 0U;
  c2_info[95].mFileTimeHi = 0U;
  c2_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[96].name = "eml_index_class";
  c2_info[96].dominantType = "";
  c2_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[96].fileTimeLo = 1323170578U;
  c2_info[96].fileTimeHi = 0U;
  c2_info[96].mFileTimeLo = 0U;
  c2_info[96].mFileTimeHi = 0U;
  c2_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[97].name = "eml_refblas_xswap";
  c2_info[97].dominantType = "double";
  c2_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[97].fileTimeLo = 1299076786U;
  c2_info[97].fileTimeHi = 0U;
  c2_info[97].mFileTimeLo = 0U;
  c2_info[97].mFileTimeHi = 0U;
  c2_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[98].name = "eml_index_class";
  c2_info[98].dominantType = "";
  c2_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[98].fileTimeLo = 1323170578U;
  c2_info[98].fileTimeHi = 0U;
  c2_info[98].mFileTimeLo = 0U;
  c2_info[98].mFileTimeHi = 0U;
  c2_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[99].name = "abs";
  c2_info[99].dominantType = "coder.internal.indexInt";
  c2_info[99].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[99].fileTimeLo = 1343830366U;
  c2_info[99].fileTimeHi = 0U;
  c2_info[99].mFileTimeLo = 0U;
  c2_info[99].mFileTimeHi = 0U;
  c2_info[100].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[100].name = "eml_scalar_abs";
  c2_info[100].dominantType = "coder.internal.indexInt";
  c2_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[100].fileTimeLo = 1286818712U;
  c2_info[100].fileTimeHi = 0U;
  c2_info[100].mFileTimeLo = 0U;
  c2_info[100].mFileTimeHi = 0U;
  c2_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[101].name = "eml_int_forloop_overflow_check";
  c2_info[101].dominantType = "";
  c2_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[101].fileTimeLo = 1346510340U;
  c2_info[101].fileTimeHi = 0U;
  c2_info[101].mFileTimeLo = 0U;
  c2_info[101].mFileTimeHi = 0U;
  c2_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[102].name = "eml_index_plus";
  c2_info[102].dominantType = "coder.internal.indexInt";
  c2_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[102].fileTimeLo = 1286818778U;
  c2_info[102].fileTimeHi = 0U;
  c2_info[102].mFileTimeLo = 0U;
  c2_info[102].mFileTimeHi = 0U;
  c2_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[103].name = "eml_div";
  c2_info[103].dominantType = "double";
  c2_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[103].fileTimeLo = 1313347810U;
  c2_info[103].fileTimeHi = 0U;
  c2_info[103].mFileTimeLo = 0U;
  c2_info[103].mFileTimeHi = 0U;
  c2_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[104].name = "eml_xgeru";
  c2_info[104].dominantType = "double";
  c2_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[104].fileTimeLo = 1299076774U;
  c2_info[104].fileTimeHi = 0U;
  c2_info[104].mFileTimeLo = 0U;
  c2_info[104].mFileTimeHi = 0U;
  c2_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[105].name = "eml_blas_inline";
  c2_info[105].dominantType = "";
  c2_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[105].fileTimeLo = 1299076768U;
  c2_info[105].fileTimeHi = 0U;
  c2_info[105].mFileTimeLo = 0U;
  c2_info[105].mFileTimeHi = 0U;
  c2_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[106].name = "eml_xger";
  c2_info[106].dominantType = "double";
  c2_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[106].fileTimeLo = 1299076774U;
  c2_info[106].fileTimeHi = 0U;
  c2_info[106].mFileTimeLo = 0U;
  c2_info[106].mFileTimeHi = 0U;
  c2_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[107].name = "eml_blas_inline";
  c2_info[107].dominantType = "";
  c2_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[107].fileTimeLo = 1299076768U;
  c2_info[107].fileTimeHi = 0U;
  c2_info[107].mFileTimeLo = 0U;
  c2_info[107].mFileTimeHi = 0U;
  c2_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[108].name = "intmax";
  c2_info[108].dominantType = "char";
  c2_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[108].fileTimeLo = 1311255316U;
  c2_info[108].fileTimeHi = 0U;
  c2_info[108].mFileTimeLo = 0U;
  c2_info[108].mFileTimeHi = 0U;
  c2_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[109].name = "min";
  c2_info[109].dominantType = "double";
  c2_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[109].fileTimeLo = 1311255318U;
  c2_info[109].fileTimeHi = 0U;
  c2_info[109].mFileTimeLo = 0U;
  c2_info[109].mFileTimeHi = 0U;
  c2_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[110].name = "eml_scalar_eg";
  c2_info[110].dominantType = "double";
  c2_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[110].fileTimeLo = 1286818796U;
  c2_info[110].fileTimeHi = 0U;
  c2_info[110].mFileTimeLo = 0U;
  c2_info[110].mFileTimeHi = 0U;
  c2_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[111].name = "eml_scalexp_alloc";
  c2_info[111].dominantType = "double";
  c2_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[111].fileTimeLo = 1352424860U;
  c2_info[111].fileTimeHi = 0U;
  c2_info[111].mFileTimeLo = 0U;
  c2_info[111].mFileTimeHi = 0U;
  c2_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[112].name = "eml_scalar_eg";
  c2_info[112].dominantType = "double";
  c2_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[112].fileTimeLo = 1286818796U;
  c2_info[112].fileTimeHi = 0U;
  c2_info[112].mFileTimeLo = 0U;
  c2_info[112].mFileTimeHi = 0U;
  c2_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[113].name = "mtimes";
  c2_info[113].dominantType = "double";
  c2_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[113].fileTimeLo = 1289519692U;
  c2_info[113].fileTimeHi = 0U;
  c2_info[113].mFileTimeLo = 0U;
  c2_info[113].mFileTimeHi = 0U;
  c2_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c2_info[114].name = "eml_index_class";
  c2_info[114].dominantType = "";
  c2_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[114].fileTimeLo = 1323170578U;
  c2_info[114].fileTimeHi = 0U;
  c2_info[114].mFileTimeLo = 0U;
  c2_info[114].mFileTimeHi = 0U;
  c2_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c2_info[115].name = "eml_refblas_xger";
  c2_info[115].dominantType = "double";
  c2_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[115].fileTimeLo = 1299076776U;
  c2_info[115].fileTimeHi = 0U;
  c2_info[115].mFileTimeLo = 0U;
  c2_info[115].mFileTimeHi = 0U;
  c2_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[116].name = "eml_refblas_xgerx";
  c2_info[116].dominantType = "char";
  c2_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[116].fileTimeLo = 1299076778U;
  c2_info[116].fileTimeHi = 0U;
  c2_info[116].mFileTimeLo = 0U;
  c2_info[116].mFileTimeHi = 0U;
  c2_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[117].name = "eml_index_class";
  c2_info[117].dominantType = "";
  c2_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[117].fileTimeLo = 1323170578U;
  c2_info[117].fileTimeHi = 0U;
  c2_info[117].mFileTimeLo = 0U;
  c2_info[117].mFileTimeHi = 0U;
  c2_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[118].name = "abs";
  c2_info[118].dominantType = "coder.internal.indexInt";
  c2_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[118].fileTimeLo = 1343830366U;
  c2_info[118].fileTimeHi = 0U;
  c2_info[118].mFileTimeLo = 0U;
  c2_info[118].mFileTimeHi = 0U;
  c2_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[119].name = "eml_index_minus";
  c2_info[119].dominantType = "double";
  c2_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[119].fileTimeLo = 1286818778U;
  c2_info[119].fileTimeHi = 0U;
  c2_info[119].mFileTimeLo = 0U;
  c2_info[119].mFileTimeHi = 0U;
  c2_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[120].name = "eml_int_forloop_overflow_check";
  c2_info[120].dominantType = "";
  c2_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[120].fileTimeLo = 1346510340U;
  c2_info[120].fileTimeHi = 0U;
  c2_info[120].mFileTimeLo = 0U;
  c2_info[120].mFileTimeHi = 0U;
  c2_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[121].name = "eml_index_plus";
  c2_info[121].dominantType = "double";
  c2_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[121].fileTimeLo = 1286818778U;
  c2_info[121].fileTimeHi = 0U;
  c2_info[121].mFileTimeLo = 0U;
  c2_info[121].mFileTimeHi = 0U;
  c2_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[122].name = "eml_index_plus";
  c2_info[122].dominantType = "coder.internal.indexInt";
  c2_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[122].fileTimeLo = 1286818778U;
  c2_info[122].fileTimeHi = 0U;
  c2_info[122].mFileTimeLo = 0U;
  c2_info[122].mFileTimeHi = 0U;
  c2_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c2_info[123].name = "eml_warning";
  c2_info[123].dominantType = "char";
  c2_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c2_info[123].fileTimeLo = 1286818802U;
  c2_info[123].fileTimeHi = 0U;
  c2_info[123].mFileTimeLo = 0U;
  c2_info[123].mFileTimeHi = 0U;
  c2_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[124].name = "eml_scalar_eg";
  c2_info[124].dominantType = "double";
  c2_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[124].fileTimeLo = 1286818796U;
  c2_info[124].fileTimeHi = 0U;
  c2_info[124].mFileTimeLo = 0U;
  c2_info[124].mFileTimeHi = 0U;
  c2_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[125].name = "eml_int_forloop_overflow_check";
  c2_info[125].dominantType = "";
  c2_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[125].fileTimeLo = 1346510340U;
  c2_info[125].fileTimeHi = 0U;
  c2_info[125].mFileTimeLo = 0U;
  c2_info[125].mFileTimeHi = 0U;
  c2_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[126].name = "eml_xtrsm";
  c2_info[126].dominantType = "char";
  c2_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c2_info[126].fileTimeLo = 1299076778U;
  c2_info[126].fileTimeHi = 0U;
  c2_info[126].mFileTimeLo = 0U;
  c2_info[126].mFileTimeHi = 0U;
  c2_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c2_info[127].name = "eml_blas_inline";
  c2_info[127].dominantType = "";
  c2_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[127].fileTimeLo = 1299076768U;
  c2_info[127].fileTimeHi = 0U;
  c2_info[127].mFileTimeLo = 0U;
  c2_info[127].mFileTimeHi = 0U;
}

static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[148])
{
  c2_info[128].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c2_info[128].name = "mtimes";
  c2_info[128].dominantType = "double";
  c2_info[128].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[128].fileTimeLo = 1289519692U;
  c2_info[128].fileTimeHi = 0U;
  c2_info[128].mFileTimeLo = 0U;
  c2_info[128].mFileTimeHi = 0U;
  c2_info[129].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[129].name = "eml_index_class";
  c2_info[129].dominantType = "";
  c2_info[129].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[129].fileTimeLo = 1323170578U;
  c2_info[129].fileTimeHi = 0U;
  c2_info[129].mFileTimeLo = 0U;
  c2_info[129].mFileTimeHi = 0U;
  c2_info[130].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[130].name = "eml_scalar_eg";
  c2_info[130].dominantType = "double";
  c2_info[130].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[130].fileTimeLo = 1286818796U;
  c2_info[130].fileTimeHi = 0U;
  c2_info[130].mFileTimeLo = 0U;
  c2_info[130].mFileTimeHi = 0U;
  c2_info[131].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[131].name = "eml_refblas_xtrsm";
  c2_info[131].dominantType = "char";
  c2_info[131].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[131].fileTimeLo = 1299076786U;
  c2_info[131].fileTimeHi = 0U;
  c2_info[131].mFileTimeLo = 0U;
  c2_info[131].mFileTimeHi = 0U;
  c2_info[132].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[132].name = "eml_scalar_eg";
  c2_info[132].dominantType = "double";
  c2_info[132].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[132].fileTimeLo = 1286818796U;
  c2_info[132].fileTimeHi = 0U;
  c2_info[132].mFileTimeLo = 0U;
  c2_info[132].mFileTimeHi = 0U;
  c2_info[133].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[133].name = "eml_index_minus";
  c2_info[133].dominantType = "double";
  c2_info[133].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[133].fileTimeLo = 1286818778U;
  c2_info[133].fileTimeHi = 0U;
  c2_info[133].mFileTimeLo = 0U;
  c2_info[133].mFileTimeHi = 0U;
  c2_info[134].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[134].name = "eml_index_class";
  c2_info[134].dominantType = "";
  c2_info[134].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[134].fileTimeLo = 1323170578U;
  c2_info[134].fileTimeHi = 0U;
  c2_info[134].mFileTimeLo = 0U;
  c2_info[134].mFileTimeHi = 0U;
  c2_info[135].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[135].name = "eml_index_times";
  c2_info[135].dominantType = "coder.internal.indexInt";
  c2_info[135].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[135].fileTimeLo = 1286818780U;
  c2_info[135].fileTimeHi = 0U;
  c2_info[135].mFileTimeLo = 0U;
  c2_info[135].mFileTimeHi = 0U;
  c2_info[136].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[136].name = "eml_index_plus";
  c2_info[136].dominantType = "coder.internal.indexInt";
  c2_info[136].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[136].fileTimeLo = 1286818778U;
  c2_info[136].fileTimeHi = 0U;
  c2_info[136].mFileTimeLo = 0U;
  c2_info[136].mFileTimeHi = 0U;
  c2_info[137].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[137].name = "eml_int_forloop_overflow_check";
  c2_info[137].dominantType = "";
  c2_info[137].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[137].fileTimeLo = 1346510340U;
  c2_info[137].fileTimeHi = 0U;
  c2_info[137].mFileTimeLo = 0U;
  c2_info[137].mFileTimeHi = 0U;
  c2_info[138].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[138].name = "eml_index_plus";
  c2_info[138].dominantType = "double";
  c2_info[138].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[138].fileTimeLo = 1286818778U;
  c2_info[138].fileTimeHi = 0U;
  c2_info[138].mFileTimeLo = 0U;
  c2_info[138].mFileTimeHi = 0U;
  c2_info[139].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[139].name = "intmin";
  c2_info[139].dominantType = "char";
  c2_info[139].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[139].fileTimeLo = 1311255318U;
  c2_info[139].fileTimeHi = 0U;
  c2_info[139].mFileTimeLo = 0U;
  c2_info[139].mFileTimeHi = 0U;
  c2_info[140].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[140].name = "eml_div";
  c2_info[140].dominantType = "double";
  c2_info[140].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[140].fileTimeLo = 1313347810U;
  c2_info[140].fileTimeHi = 0U;
  c2_info[140].mFileTimeLo = 0U;
  c2_info[140].mFileTimeHi = 0U;
  c2_info[141].context = "";
  c2_info[141].name = "acos";
  c2_info[141].dominantType = "double";
  c2_info[141].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/acos.m";
  c2_info[141].fileTimeLo = 1343830366U;
  c2_info[141].fileTimeHi = 0U;
  c2_info[141].mFileTimeLo = 0U;
  c2_info[141].mFileTimeHi = 0U;
  c2_info[142].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/acos.m";
  c2_info[142].name = "eml_error";
  c2_info[142].dominantType = "char";
  c2_info[142].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[142].fileTimeLo = 1343830358U;
  c2_info[142].fileTimeHi = 0U;
  c2_info[142].mFileTimeLo = 0U;
  c2_info[142].mFileTimeHi = 0U;
  c2_info[143].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/acos.m";
  c2_info[143].name = "eml_scalar_acos";
  c2_info[143].dominantType = "double";
  c2_info[143].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_acos.m";
  c2_info[143].fileTimeLo = 1343830376U;
  c2_info[143].fileTimeHi = 0U;
  c2_info[143].mFileTimeLo = 0U;
  c2_info[143].mFileTimeHi = 0U;
  c2_info[144].context = "";
  c2_info[144].name = "mtimes";
  c2_info[144].dominantType = "double";
  c2_info[144].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[144].fileTimeLo = 1289519692U;
  c2_info[144].fileTimeHi = 0U;
  c2_info[144].mFileTimeLo = 0U;
  c2_info[144].mFileTimeHi = 0U;
  c2_info[145].context = "";
  c2_info[145].name = "mrdivide";
  c2_info[145].dominantType = "double";
  c2_info[145].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[145].fileTimeLo = 1357951548U;
  c2_info[145].fileTimeHi = 0U;
  c2_info[145].mFileTimeLo = 1319729966U;
  c2_info[145].mFileTimeHi = 0U;
  c2_info[146].context = "";
  c2_info[146].name = "mpower";
  c2_info[146].dominantType = "double";
  c2_info[146].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[146].fileTimeLo = 1286818842U;
  c2_info[146].fileTimeHi = 0U;
  c2_info[146].mFileTimeLo = 0U;
  c2_info[146].mFileTimeHi = 0U;
  c2_info[147].context = "";
  c2_info[147].name = "sqrt";
  c2_info[147].dominantType = "double";
  c2_info[147].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[147].fileTimeLo = 1343830386U;
  c2_info[147].fileTimeHi = 0U;
  c2_info[147].mFileTimeLo = 0U;
  c2_info[147].mFileTimeHi = 0U;
}

static void c2_eml_scalar_eg(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance)
{
}

static real_T c2_mpower(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_ak;
  real_T c2_d_a;
  real_T c2_e_a;
  real_T c2_b;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_b_eml_scalar_eg(chartInstance);
  c2_ak = c2_c_a;
  c2_d_a = c2_ak;
  c2_b_eml_scalar_eg(chartInstance);
  c2_e_a = c2_d_a;
  c2_b = c2_d_a;
  return c2_e_a * c2_b;
}

static void c2_b_eml_scalar_eg(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance)
{
}

static void c2_eml_error(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
  int32_T c2_i76;
  static char_T c2_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  int32_T c2_i77;
  static char_T c2_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c2_b_u[4];
  const mxArray *c2_b_y = NULL;
  for (c2_i76 = 0; c2_i76 < 30; c2_i76++) {
    c2_u[c2_i76] = c2_cv0[c2_i76];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  for (c2_i77 = 0; c2_i77 < 4; c2_i77++) {
    c2_b_u[c2_i77] = c2_cv1[c2_i77];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c2_y, 14, c2_b_y));
}

static void c2_realmin(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
}

static void c2_eps(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance)
{
}

static void c2_eml_matlab_zgetrf(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_b_A[16], int32_T c2_ipiv[4],
  int32_T *c2_info)
{
  int32_T c2_i78;
  for (c2_i78 = 0; c2_i78 < 16; c2_i78++) {
    c2_b_A[c2_i78] = c2_A[c2_i78];
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_b_A, c2_ipiv, c2_info);
}

static void c2_check_forloop_overflow_error
  (SFc2_Thrust_Direction_quadInstanceStruct *chartInstance, boolean_T
   c2_overflow)
{
  int32_T c2_i79;
  static char_T c2_cv2[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c2_u[34];
  const mxArray *c2_y = NULL;
  int32_T c2_i80;
  static char_T c2_cv3[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c2_b_u[23];
  const mxArray *c2_b_y = NULL;
  if (!c2_overflow) {
  } else {
    for (c2_i79 = 0; c2_i79 < 34; c2_i79++) {
      c2_u[c2_i79] = c2_cv2[c2_i79];
    }

    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c2_i80 = 0; c2_i80 < 23; c2_i80++) {
      c2_b_u[c2_i80] = c2_cv3[c2_i80];
    }

    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c2_y, 14, c2_b_y));
  }
}

static void c2_eml_xger(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0,
  real_T c2_A[16], int32_T c2_ia0, real_T c2_b_A[16])
{
  int32_T c2_i81;
  for (c2_i81 = 0; c2_i81 < 16; c2_i81++) {
    c2_b_A[c2_i81] = c2_A[c2_i81];
  }

  c2_b_eml_xger(chartInstance, c2_m, c2_n, c2_alpha1, c2_ix0, c2_iy0, c2_b_A,
                c2_ia0);
}

static void c2_eml_warning(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance)
{
  int32_T c2_i82;
  static char_T c2_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c2_u[27];
  const mxArray *c2_y = NULL;
  for (c2_i82 = 0; c2_i82 < 27; c2_i82++) {
    c2_u[c2_i82] = c2_varargin_1[c2_i82];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c2_y));
}

static void c2_eml_xtrsm(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  real_T c2_A[16], real_T c2_B[4], real_T c2_b_B[4])
{
  int32_T c2_i83;
  int32_T c2_i84;
  real_T c2_b_A[16];
  for (c2_i83 = 0; c2_i83 < 4; c2_i83++) {
    c2_b_B[c2_i83] = c2_B[c2_i83];
  }

  for (c2_i84 = 0; c2_i84 < 16; c2_i84++) {
    c2_b_A[c2_i84] = c2_A[c2_i84];
  }

  c2_c_eml_xtrsm(chartInstance, c2_b_A, c2_b_B);
}

static void c2_below_threshold(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance)
{
}

static void c2_c_eml_scalar_eg(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance)
{
}

static void c2_b_eml_xtrsm(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[4], real_T c2_b_B[4])
{
  int32_T c2_i85;
  int32_T c2_i86;
  real_T c2_b_A[16];
  for (c2_i85 = 0; c2_i85 < 4; c2_i85++) {
    c2_b_B[c2_i85] = c2_B[c2_i85];
  }

  for (c2_i86 = 0; c2_i86 < 16; c2_i86++) {
    c2_b_A[c2_i86] = c2_A[c2_i86];
  }

  c2_d_eml_xtrsm(chartInstance, c2_b_A, c2_b_B);
}

static void c2_b_eml_error(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance)
{
  int32_T c2_i87;
  static char_T c2_cv4[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  int32_T c2_i88;
  static char_T c2_cv5[4] = { 'a', 'c', 'o', 's' };

  char_T c2_b_u[4];
  const mxArray *c2_b_y = NULL;
  for (c2_i87 = 0; c2_i87 < 30; c2_i87++) {
    c2_u[c2_i87] = c2_cv4[c2_i87];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  for (c2_i88 = 0; c2_i88 < 4; c2_i88++) {
    c2_b_u[c2_i88] = c2_cv5[c2_i88];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c2_y, 14, c2_b_y));
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_i_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i89;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i89, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i89;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_j_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_Thrust_Direction_quad, const
  char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_Thrust_Direction_quad), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_Thrust_Direction_quad);
  return c2_y;
}

static uint8_T c2_k_emlrt_marshallIn(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_eml_matlab_zgetrf(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_A[16], int32_T c2_ipiv[4], int32_T *c2_info)
{
  int32_T c2_i90;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_jm1;
  int32_T c2_b;
  int32_T c2_mmj;
  int32_T c2_b_a;
  int32_T c2_c;
  int32_T c2_b_b;
  int32_T c2_jj;
  int32_T c2_c_a;
  int32_T c2_jp1j;
  int32_T c2_d_a;
  int32_T c2_b_c;
  int32_T c2_n;
  int32_T c2_ix0;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_idxmax;
  int32_T c2_ix;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_y;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_b_y;
  real_T c2_smax;
  int32_T c2_d_n;
  int32_T c2_c_b;
  int32_T c2_d_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_e_a;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_c_y;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_d_y;
  real_T c2_s;
  int32_T c2_f_a;
  int32_T c2_jpiv_offset;
  int32_T c2_g_a;
  int32_T c2_e_b;
  int32_T c2_jpiv;
  int32_T c2_h_a;
  int32_T c2_f_b;
  int32_T c2_c_c;
  int32_T c2_g_b;
  int32_T c2_jrow;
  int32_T c2_i_a;
  int32_T c2_h_b;
  int32_T c2_jprow;
  int32_T c2_d_ix0;
  int32_T c2_iy0;
  int32_T c2_e_ix0;
  int32_T c2_b_iy0;
  int32_T c2_f_ix0;
  int32_T c2_c_iy0;
  int32_T c2_b_ix;
  int32_T c2_iy;
  int32_T c2_c_k;
  real_T c2_temp;
  int32_T c2_j_a;
  int32_T c2_k_a;
  int32_T c2_b_jp1j;
  int32_T c2_l_a;
  int32_T c2_d_c;
  int32_T c2_m_a;
  int32_T c2_i_b;
  int32_T c2_i91;
  int32_T c2_n_a;
  int32_T c2_j_b;
  int32_T c2_o_a;
  int32_T c2_k_b;
  boolean_T c2_b_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_k_x;
  real_T c2_e_y;
  real_T c2_z;
  int32_T c2_l_b;
  int32_T c2_e_c;
  int32_T c2_p_a;
  int32_T c2_f_c;
  int32_T c2_q_a;
  int32_T c2_g_c;
  int32_T c2_m;
  int32_T c2_e_n;
  int32_T c2_g_ix0;
  int32_T c2_d_iy0;
  int32_T c2_ia0;
  real_T c2_d1;
  c2_realmin(chartInstance);
  c2_eps(chartInstance);
  for (c2_i90 = 0; c2_i90 < 4; c2_i90++) {
    c2_ipiv[c2_i90] = 1 + c2_i90;
  }

  *c2_info = 0;
  for (c2_j = 1; c2_j < 4; c2_j++) {
    c2_b_j = c2_j;
    c2_a = c2_b_j - 1;
    c2_jm1 = c2_a;
    c2_b = c2_b_j;
    c2_mmj = 4 - c2_b;
    c2_b_a = c2_jm1;
    c2_c = c2_b_a * 5;
    c2_b_b = c2_c + 1;
    c2_jj = c2_b_b;
    c2_c_a = c2_jj + 1;
    c2_jp1j = c2_c_a;
    c2_d_a = c2_mmj;
    c2_b_c = c2_d_a;
    c2_n = c2_b_c + 1;
    c2_ix0 = c2_jj;
    c2_b_n = c2_n;
    c2_b_ix0 = c2_ix0;
    c2_c_n = c2_b_n;
    c2_c_ix0 = c2_b_ix0;
    if (c2_c_n < 1) {
      c2_idxmax = 0;
    } else {
      c2_idxmax = 1;
      if (c2_c_n > 1) {
        c2_ix = c2_c_ix0;
        c2_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_ix), 1, 16, 1, 0) - 1];
        c2_b_x = c2_x;
        c2_c_x = c2_b_x;
        c2_y = muDoubleScalarAbs(c2_c_x);
        c2_d_x = 0.0;
        c2_e_x = c2_d_x;
        c2_b_y = muDoubleScalarAbs(c2_e_x);
        c2_smax = c2_y + c2_b_y;
        c2_d_n = c2_c_n;
        c2_c_b = c2_d_n;
        c2_d_b = c2_c_b;
        if (2 > c2_d_b) {
          c2_overflow = FALSE;
        } else {
          c2_overflow = (c2_d_b > 2147483646);
        }

        if (c2_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_overflow);
        }

        for (c2_k = 2; c2_k <= c2_d_n; c2_k++) {
          c2_b_k = c2_k;
          c2_e_a = c2_ix + 1;
          c2_ix = c2_e_a;
          c2_f_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_ix), 1, 16, 1, 0) - 1];
          c2_g_x = c2_f_x;
          c2_h_x = c2_g_x;
          c2_c_y = muDoubleScalarAbs(c2_h_x);
          c2_i_x = 0.0;
          c2_j_x = c2_i_x;
          c2_d_y = muDoubleScalarAbs(c2_j_x);
          c2_s = c2_c_y + c2_d_y;
          if (c2_s > c2_smax) {
            c2_idxmax = c2_b_k;
            c2_smax = c2_s;
          }
        }
      }
    }

    c2_f_a = c2_idxmax - 1;
    c2_jpiv_offset = c2_f_a;
    c2_g_a = c2_jj;
    c2_e_b = c2_jpiv_offset;
    c2_jpiv = c2_g_a + c2_e_b;
    if (c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_jpiv), 1, 16, 1, 0) - 1] != 0.0) {
      if (c2_jpiv_offset != 0) {
        c2_h_a = c2_b_j;
        c2_f_b = c2_jpiv_offset;
        c2_c_c = c2_h_a + c2_f_b;
        c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j), 1, 4, 1, 0) - 1] = c2_c_c;
        c2_g_b = c2_jm1 + 1;
        c2_jrow = c2_g_b;
        c2_i_a = c2_jrow;
        c2_h_b = c2_jpiv_offset;
        c2_jprow = c2_i_a + c2_h_b;
        c2_d_ix0 = c2_jrow;
        c2_iy0 = c2_jprow;
        c2_e_ix0 = c2_d_ix0;
        c2_b_iy0 = c2_iy0;
        c2_f_ix0 = c2_e_ix0;
        c2_c_iy0 = c2_b_iy0;
        c2_b_ix = c2_f_ix0;
        c2_iy = c2_c_iy0;
        for (c2_c_k = 1; c2_c_k < 5; c2_c_k++) {
          c2_temp = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 16, 1, 0) - 1];
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ix), 1, 16, 1, 0) - 1] =
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_iy), 1, 16, 1, 0) - 1];
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_iy), 1, 16, 1, 0) - 1] = c2_temp;
          c2_j_a = c2_b_ix + 4;
          c2_b_ix = c2_j_a;
          c2_k_a = c2_iy + 4;
          c2_iy = c2_k_a;
        }
      }

      c2_b_jp1j = c2_jp1j;
      c2_l_a = c2_mmj;
      c2_d_c = c2_l_a;
      c2_m_a = c2_jp1j;
      c2_i_b = c2_d_c - 1;
      c2_i91 = c2_m_a + c2_i_b;
      c2_n_a = c2_b_jp1j;
      c2_j_b = c2_i91;
      c2_o_a = c2_n_a;
      c2_k_b = c2_j_b;
      if (c2_o_a > c2_k_b) {
        c2_b_overflow = FALSE;
      } else {
        c2_b_overflow = (c2_k_b > 2147483646);
      }

      if (c2_b_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
      }

      for (c2_i = c2_b_jp1j; c2_i <= c2_i91; c2_i++) {
        c2_b_i = c2_i;
        c2_k_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 16, 1, 0) - 1];
        c2_e_y = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_jj), 1, 16, 1, 0) - 1];
        c2_z = c2_k_x / c2_e_y;
        c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_i), 1, 16, 1, 0) - 1] = c2_z;
      }
    } else {
      *c2_info = c2_b_j;
    }

    c2_l_b = c2_b_j;
    c2_e_c = 4 - c2_l_b;
    c2_p_a = c2_jj;
    c2_f_c = c2_p_a;
    c2_q_a = c2_jj;
    c2_g_c = c2_q_a;
    c2_m = c2_mmj;
    c2_e_n = c2_e_c;
    c2_g_ix0 = c2_jp1j;
    c2_d_iy0 = c2_f_c + 4;
    c2_ia0 = c2_g_c + 5;
    c2_d1 = -1.0;
    c2_b_eml_xger(chartInstance, c2_m, c2_e_n, c2_d1, c2_g_ix0, c2_d_iy0, c2_A,
                  c2_ia0);
  }

  if (*c2_info == 0) {
    if (!(c2_A[15] != 0.0)) {
      *c2_info = 4;
    }
  }
}

static void c2_b_eml_xger(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0,
  int32_T c2_iy0, real_T c2_A[16], int32_T c2_ia0)
{
  int32_T c2_b_m;
  int32_T c2_b_n;
  real_T c2_b_alpha1;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_b_ia0;
  int32_T c2_c_m;
  int32_T c2_c_n;
  real_T c2_c_alpha1;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_c_ia0;
  int32_T c2_d_m;
  int32_T c2_d_n;
  real_T c2_d_alpha1;
  int32_T c2_d_ix0;
  int32_T c2_d_iy0;
  int32_T c2_d_ia0;
  int32_T c2_ixstart;
  int32_T c2_a;
  int32_T c2_jA;
  int32_T c2_jy;
  int32_T c2_e_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_j;
  real_T c2_yjy;
  real_T c2_temp;
  int32_T c2_ix;
  int32_T c2_c_b;
  int32_T c2_i92;
  int32_T c2_b_a;
  int32_T c2_d_b;
  int32_T c2_i93;
  int32_T c2_c_a;
  int32_T c2_e_b;
  int32_T c2_d_a;
  int32_T c2_f_b;
  boolean_T c2_b_overflow;
  int32_T c2_ijA;
  int32_T c2_b_ijA;
  int32_T c2_e_a;
  int32_T c2_f_a;
  int32_T c2_g_a;
  c2_b_m = c2_m;
  c2_b_n = c2_n;
  c2_b_alpha1 = c2_alpha1;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_b_ia0 = c2_ia0;
  c2_c_m = c2_b_m;
  c2_c_n = c2_b_n;
  c2_c_alpha1 = c2_b_alpha1;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_c_ia0 = c2_b_ia0;
  c2_d_m = c2_c_m;
  c2_d_n = c2_c_n;
  c2_d_alpha1 = c2_c_alpha1;
  c2_d_ix0 = c2_c_ix0;
  c2_d_iy0 = c2_c_iy0;
  c2_d_ia0 = c2_c_ia0;
  if (c2_d_alpha1 == 0.0) {
  } else {
    c2_ixstart = c2_d_ix0;
    c2_a = c2_d_ia0 - 1;
    c2_jA = c2_a;
    c2_jy = c2_d_iy0;
    c2_e_n = c2_d_n;
    c2_b = c2_e_n;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_j = 1; c2_j <= c2_e_n; c2_j++) {
      c2_yjy = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_jy), 1, 16, 1, 0) - 1];
      if (c2_yjy != 0.0) {
        c2_temp = c2_yjy * c2_d_alpha1;
        c2_ix = c2_ixstart;
        c2_c_b = c2_jA + 1;
        c2_i92 = c2_c_b;
        c2_b_a = c2_d_m;
        c2_d_b = c2_jA;
        c2_i93 = c2_b_a + c2_d_b;
        c2_c_a = c2_i92;
        c2_e_b = c2_i93;
        c2_d_a = c2_c_a;
        c2_f_b = c2_e_b;
        if (c2_d_a > c2_f_b) {
          c2_b_overflow = FALSE;
        } else {
          c2_b_overflow = (c2_f_b > 2147483646);
        }

        if (c2_b_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
        }

        for (c2_ijA = c2_i92; c2_ijA <= c2_i93; c2_ijA++) {
          c2_b_ijA = c2_ijA;
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ijA), 1, 16, 1, 0) - 1] =
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ijA), 1, 16, 1, 0) - 1] +
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_ix), 1, 16, 1, 0) - 1] * c2_temp;
          c2_e_a = c2_ix + 1;
          c2_ix = c2_e_a;
        }
      }

      c2_f_a = c2_jy + 4;
      c2_jy = c2_f_a;
      c2_g_a = c2_jA + 4;
      c2_jA = c2_g_a;
    }
  }
}

static void c2_c_eml_xtrsm(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[4])
{
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_b_b;
  int32_T c2_kAcol;
  int32_T c2_b_a;
  int32_T c2_c_c;
  int32_T c2_c_a;
  int32_T c2_i94;
  boolean_T c2_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_d_a;
  int32_T c2_d_c;
  int32_T c2_e_a;
  int32_T c2_e_c;
  int32_T c2_f_a;
  int32_T c2_f_c;
  int32_T c2_g_a;
  int32_T c2_c_b;
  int32_T c2_g_c;
  c2_below_threshold(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  for (c2_k = 1; c2_k < 5; c2_k++) {
    c2_b_k = c2_k;
    c2_a = c2_b_k;
    c2_c = c2_a;
    c2_b = c2_c - 1;
    c2_b_c = c2_b << 2;
    c2_b_b = c2_b_c;
    c2_kAcol = c2_b_b;
    c2_b_a = c2_b_k;
    c2_c_c = c2_b_a;
    if (c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_c_c), 1, 4, 1, 0) - 1] != 0.0) {
      c2_c_a = c2_b_k;
      c2_i94 = c2_c_a;
      c2_overflow = FALSE;
      if (c2_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_overflow);
      }

      for (c2_i = c2_i94 + 1; c2_i < 5; c2_i++) {
        c2_b_i = c2_i;
        c2_d_a = c2_b_i;
        c2_d_c = c2_d_a;
        c2_e_a = c2_b_i;
        c2_e_c = c2_e_a;
        c2_f_a = c2_b_k;
        c2_f_c = c2_f_a;
        c2_g_a = c2_b_i;
        c2_c_b = c2_kAcol;
        c2_g_c = c2_g_a + c2_c_b;
        c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_d_c), 1, 4, 1, 0) - 1] = c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_e_c), 1, 4, 1, 0) - 1]
          - c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_f_c), 1, 4, 1, 0) - 1] * c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_g_c), 1, 16, 1, 0) - 1];
      }
    }
  }
}

static void c2_d_eml_xtrsm(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[4])
{
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_b_b;
  int32_T c2_kAcol;
  int32_T c2_b_a;
  int32_T c2_c_c;
  int32_T c2_c_a;
  int32_T c2_d_c;
  int32_T c2_d_a;
  int32_T c2_e_c;
  int32_T c2_e_a;
  int32_T c2_c_b;
  int32_T c2_f_c;
  real_T c2_x;
  real_T c2_y;
  real_T c2_z;
  int32_T c2_f_a;
  int32_T c2_i95;
  int32_T c2_d_b;
  int32_T c2_e_b;
  boolean_T c2_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_g_a;
  int32_T c2_g_c;
  int32_T c2_h_a;
  int32_T c2_h_c;
  int32_T c2_i_a;
  int32_T c2_i_c;
  int32_T c2_j_a;
  int32_T c2_f_b;
  int32_T c2_j_c;
  c2_below_threshold(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  for (c2_k = 4; c2_k > 0; c2_k--) {
    c2_b_k = c2_k;
    c2_a = c2_b_k;
    c2_c = c2_a;
    c2_b = c2_c - 1;
    c2_b_c = c2_b << 2;
    c2_b_b = c2_b_c;
    c2_kAcol = c2_b_b;
    c2_b_a = c2_b_k;
    c2_c_c = c2_b_a;
    if (c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_c_c), 1, 4, 1, 0) - 1] != 0.0) {
      c2_c_a = c2_b_k;
      c2_d_c = c2_c_a;
      c2_d_a = c2_b_k;
      c2_e_c = c2_d_a;
      c2_e_a = c2_b_k;
      c2_c_b = c2_kAcol;
      c2_f_c = c2_e_a + c2_c_b;
      c2_x = c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_e_c), 1, 4, 1, 0) - 1];
      c2_y = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_f_c), 1, 16, 1, 0) - 1];
      c2_z = c2_x / c2_y;
      c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_d_c), 1, 4, 1, 0) - 1] = c2_z;
      c2_f_a = c2_b_k - 1;
      c2_i95 = c2_f_a;
      c2_d_b = c2_i95;
      c2_e_b = c2_d_b;
      if (1 > c2_e_b) {
        c2_overflow = FALSE;
      } else {
        c2_overflow = (c2_e_b > 2147483646);
      }

      if (c2_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_overflow);
      }

      for (c2_i = 1; c2_i <= c2_i95; c2_i++) {
        c2_b_i = c2_i;
        c2_g_a = c2_b_i;
        c2_g_c = c2_g_a;
        c2_h_a = c2_b_i;
        c2_h_c = c2_h_a;
        c2_i_a = c2_b_k;
        c2_i_c = c2_i_a;
        c2_j_a = c2_b_i;
        c2_f_b = c2_kAcol;
        c2_j_c = c2_j_a + c2_f_b;
        c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_g_c), 1, 4, 1, 0) - 1] = c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_h_c), 1, 4, 1, 0) - 1]
          - c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_i_c), 1, 4, 1, 0) - 1] * c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_j_c), 1, 16, 1, 0) - 1];
      }
    }
  }
}

static real_T c2_get_J_x(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 0, NULL, c2_b);
  if (chartInstance->c2_J_x_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'J_x\' (#357) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_J_x_address;
}

static void c2_set_J_x(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 0, NULL, c2_b);
  if (chartInstance->c2_J_x_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'J_x\' (#357) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_J_x_address = c2_c;
}

static real_T *c2_access_J_x(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 0, NULL);
  if (chartInstance->c2_J_x_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'J_x\' (#357) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_J_x_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 0, NULL);
  }

  return c2_c;
}

static real_T c2_get_J_y(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 1, NULL, c2_b);
  if (chartInstance->c2_J_y_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'J_y\' (#358) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_J_y_address;
}

static void c2_set_J_y(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 1, NULL, c2_b);
  if (chartInstance->c2_J_y_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'J_y\' (#358) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_J_y_address = c2_c;
}

static real_T *c2_access_J_y(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 1, NULL);
  if (chartInstance->c2_J_y_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'J_y\' (#358) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_J_y_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 1, NULL);
  }

  return c2_c;
}

static real_T c2_get_J_z(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 2, NULL, c2_b);
  if (chartInstance->c2_J_z_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'J_z\' (#359) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_J_z_address;
}

static void c2_set_J_z(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 2, NULL, c2_b);
  if (chartInstance->c2_J_z_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'J_z\' (#359) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_J_z_address = c2_c;
}

static real_T *c2_access_J_z(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 2, NULL);
  if (chartInstance->c2_J_z_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'J_z\' (#359) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_J_z_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 2, NULL);
  }

  return c2_c;
}

static real_T c2_get_brake_torque(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 3, NULL, c2_b);
  if (chartInstance->c2_brake_torque_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'brake_torque\' (#354) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_brake_torque_address;
}

static void c2_set_brake_torque(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 3, NULL, c2_b);
  if (chartInstance->c2_brake_torque_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'brake_torque\' (#354) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_brake_torque_address = c2_c;
}

static real_T *c2_access_brake_torque(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 3, NULL);
  if (chartInstance->c2_brake_torque_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'brake_torque\' (#354) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_brake_torque_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 3, NULL);
  }

  return c2_c;
}

static real_T c2_get_c_phi(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 4, NULL, c2_b);
  if (chartInstance->c2_c_phi_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'c_phi\' (#349) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_c_phi_address;
}

static void c2_set_c_phi(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 4, NULL, c2_b);
  if (chartInstance->c2_c_phi_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'c_phi\' (#349) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_c_phi_address = c2_c;
}

static real_T *c2_access_c_phi(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 4, NULL);
  if (chartInstance->c2_c_phi_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'c_phi\' (#349) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_c_phi_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 4, NULL);
  }

  return c2_c;
}

static real_T c2_get_d(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 5, NULL, c2_b);
  if (chartInstance->c2_d_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'d\' (#362) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_d_address;
}

static void c2_set_d(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
                     uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 5, NULL, c2_b);
  if (chartInstance->c2_d_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'d\' (#362) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_d_address = c2_c;
}

static real_T *c2_access_d(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 5, NULL);
  if (chartInstance->c2_d_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'d\' (#362) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_d_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 5, NULL);
  }

  return c2_c;
}

static real_T c2_get_d_l(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 6, NULL, c2_b);
  if (chartInstance->c2_d_l_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'d_l\' (#351) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_d_l_address;
}

static void c2_set_d_l(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 6, NULL, c2_b);
  if (chartInstance->c2_d_l_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'d_l\' (#351) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_d_l_address = c2_c;
}

static real_T *c2_access_d_l(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 6, NULL);
  if (chartInstance->c2_d_l_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'d_l\' (#351) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_d_l_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 6, NULL);
  }

  return c2_c;
}

static real_T c2_get_d_z(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 7, NULL, c2_b);
  if (chartInstance->c2_d_z_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'d_z\' (#364) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_d_z_address;
}

static void c2_set_d_z(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 7, NULL, c2_b);
  if (chartInstance->c2_d_z_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'d_z\' (#364) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_d_z_address = c2_c;
}

static real_T *c2_access_d_z(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 7, NULL);
  if (chartInstance->c2_d_z_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'d_z\' (#364) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_d_z_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 7, NULL);
  }

  return c2_c;
}

static real_T c2_get_delta_phi(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 8, NULL, c2_b);
  if (chartInstance->c2_delta_phi_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'delta_phi\' (#361) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_delta_phi_address;
}

static void c2_set_delta_phi(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 8, NULL, c2_b);
  if (chartInstance->c2_delta_phi_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'delta_phi\' (#361) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_delta_phi_address = c2_c;
}

static real_T *c2_access_delta_phi(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 8, NULL);
  if (chartInstance->c2_delta_phi_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'delta_phi\' (#361) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_delta_phi_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 8, NULL);
  }

  return c2_c;
}

static real_T c2_get_phi_low(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 9, NULL, c2_b);
  if (chartInstance->c2_phi_low_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'phi_low\' (#348) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_phi_low_address;
}

static void c2_set_phi_low(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 9, NULL, c2_b);
  if (chartInstance->c2_phi_low_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'phi_low\' (#348) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_phi_low_address = c2_c;
}

static real_T *c2_access_phi_low(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 9, NULL);
  if (chartInstance->c2_phi_low_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'phi_low\' (#348) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_phi_low_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 9, NULL);
  }

  return c2_c;
}

static real_T c2_get_phi_up(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 10, NULL, c2_b);
  if (chartInstance->c2_phi_up_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'phi_up\' (#350) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_phi_up_address;
}

static void c2_set_phi_up(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 10, NULL, c2_b);
  if (chartInstance->c2_phi_up_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'phi_up\' (#350) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_phi_up_address = c2_c;
}

static real_T *c2_access_phi_up(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 10, NULL);
  if (chartInstance->c2_phi_up_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'phi_up\' (#350) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_phi_up_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 10, NULL);
  }

  return c2_c;
}

static real_T c2_get_r(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 11, NULL, c2_b);
  if (chartInstance->c2_r_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'r\' (#360) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_r_address;
}

static void c2_set_r(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
                     uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 11, NULL, c2_b);
  if (chartInstance->c2_r_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'r\' (#360) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_r_address = c2_c;
}

static real_T *c2_access_r(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 11, NULL);
  if (chartInstance->c2_r_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'r\' (#360) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_r_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 11, NULL);
  }

  return c2_c;
}

static real_T c2_get_t_s(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 12, NULL, c2_b);
  if (chartInstance->c2_t_s_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'t_s\' (#356) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_t_s_address;
}

static void c2_set_t_s(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 12, NULL, c2_b);
  if (chartInstance->c2_t_s_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'t_s\' (#356) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_t_s_address = c2_c;
}

static real_T *c2_access_t_s(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 12, NULL);
  if (chartInstance->c2_t_s_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'t_s\' (#356) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_t_s_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 12, NULL);
  }

  return c2_c;
}

static real_T c2_get_torque_xy(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 13, NULL, c2_b);
  if (chartInstance->c2_torque_xy_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'torque_xy\' (#355) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_torque_xy_address;
}

static void c2_set_torque_xy(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 13, NULL, c2_b);
  if (chartInstance->c2_torque_xy_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'torque_xy\' (#355) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_torque_xy_address = c2_c;
}

static real_T *c2_access_torque_xy(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 13, NULL);
  if (chartInstance->c2_torque_xy_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'torque_xy\' (#355) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_torque_xy_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 13, NULL);
  }

  return c2_c;
}

static real_T c2_get_torque_z(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 14, NULL, c2_b);
  if (chartInstance->c2_torque_z_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'torque_z\' (#363) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_torque_z_address;
}

static void c2_set_torque_z(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 14, NULL, c2_b);
  if (chartInstance->c2_torque_z_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'torque_z\' (#363) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_torque_z_address = c2_c;
}

static real_T *c2_access_torque_z(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 14, NULL);
  if (chartInstance->c2_torque_z_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'torque_z\' (#363) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_torque_z_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 14, NULL);
  }

  return c2_c;
}

static real_T c2_get_v_1(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 15, NULL, c2_b);
  if (chartInstance->c2_v_1_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'v_1\' (#352) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_v_1_address;
}

static void c2_set_v_1(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 15, NULL, c2_b);
  if (chartInstance->c2_v_1_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'v_1\' (#352) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_v_1_address = c2_c;
}

static real_T *c2_access_v_1(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 15, NULL);
  if (chartInstance->c2_v_1_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'v_1\' (#352) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_v_1_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 15, NULL);
  }

  return c2_c;
}

static real_T c2_get_v_2(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 16, NULL, c2_b);
  if (chartInstance->c2_v_2_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'v_2\' (#353) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_v_2_address;
}

static void c2_set_v_2(SFc2_Thrust_Direction_quadInstanceStruct *chartInstance,
  uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 16, NULL, c2_b);
  if (chartInstance->c2_v_2_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'v_2\' (#353) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_v_2_address = c2_c;
}

static real_T *c2_access_v_2(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance, uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 16, NULL);
  if (chartInstance->c2_v_2_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'v_2\' (#353) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_v_2_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 16, NULL);
  }

  return c2_c;
}

static void init_dsm_address_info(SFc2_Thrust_Direction_quadInstanceStruct
  *chartInstance)
{
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "J_x", (void **)
    &chartInstance->c2_J_x_address, &chartInstance->c2_J_x_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "J_y", (void **)
    &chartInstance->c2_J_y_address, &chartInstance->c2_J_y_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "J_z", (void **)
    &chartInstance->c2_J_z_address, &chartInstance->c2_J_z_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "brake_torque", (void **)
    &chartInstance->c2_brake_torque_address,
    &chartInstance->c2_brake_torque_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "c_phi", (void **)
    &chartInstance->c2_c_phi_address, &chartInstance->c2_c_phi_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "d", (void **)
    &chartInstance->c2_d_address, &chartInstance->c2_d_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "d_l", (void **)
    &chartInstance->c2_d_l_address, &chartInstance->c2_d_l_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "d_z", (void **)
    &chartInstance->c2_d_z_address, &chartInstance->c2_d_z_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "delta_phi", (void **)
    &chartInstance->c2_delta_phi_address, &chartInstance->c2_delta_phi_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "phi_low", (void **)
    &chartInstance->c2_phi_low_address, &chartInstance->c2_phi_low_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "phi_up", (void **)
    &chartInstance->c2_phi_up_address, &chartInstance->c2_phi_up_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "r", (void **)
    &chartInstance->c2_r_address, &chartInstance->c2_r_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "t_s", (void **)
    &chartInstance->c2_t_s_address, &chartInstance->c2_t_s_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "torque_xy", (void **)
    &chartInstance->c2_torque_xy_address, &chartInstance->c2_torque_xy_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "torque_z", (void **)
    &chartInstance->c2_torque_z_address, &chartInstance->c2_torque_z_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "v_1", (void **)
    &chartInstance->c2_v_1_address, &chartInstance->c2_v_1_index);
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "v_2", (void **)
    &chartInstance->c2_v_2_address, &chartInstance->c2_v_2_index);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_Thrust_Direction_quad_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2875485563U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(687150788U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1124041436U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1894746460U);
}

mxArray *sf_c2_Thrust_Direction_quad_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("lCk7drFuNGfR2YQVmiuLv");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_Thrust_Direction_quad_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c2_Thrust_Direction_quad(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[5],T\"Torques\",},{M[1],M[9],T\"phi\",},{M[8],M[0],T\"is_active_c2_Thrust_Direction_quad\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_Thrust_Direction_quad_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
    chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _Thrust_Direction_quadMachineNumber_,
           2,
           1,
           1,
           23,
           0,
           0,
           0,
           0,
           6,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_Thrust_Direction_quadMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_Thrust_Direction_quadMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _Thrust_Direction_quadMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"Orientation");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Torques");
          _SFD_SET_DATA_PROPS(2,1,1,0,"Desired_Orientation");
          _SFD_SET_DATA_PROPS(3,1,1,0,"Angular_Vel");
          _SFD_SET_DATA_PROPS(4,1,1,0,"last_phi");
          _SFD_SET_DATA_PROPS(5,2,0,1,"phi");
          _SFD_SET_DATA_PROPS(6,11,0,0,"phi_low");
          _SFD_SET_DATA_PROPS(7,11,0,0,"c_phi");
          _SFD_SET_DATA_PROPS(8,11,0,0,"phi_up");
          _SFD_SET_DATA_PROPS(9,11,0,0,"d_l");
          _SFD_SET_DATA_PROPS(10,11,0,0,"v_1");
          _SFD_SET_DATA_PROPS(11,11,0,0,"v_2");
          _SFD_SET_DATA_PROPS(12,11,0,0,"brake_torque");
          _SFD_SET_DATA_PROPS(13,11,0,0,"torque_xy");
          _SFD_SET_DATA_PROPS(14,11,0,0,"t_s");
          _SFD_SET_DATA_PROPS(15,11,0,0,"J_x");
          _SFD_SET_DATA_PROPS(16,11,0,0,"J_y");
          _SFD_SET_DATA_PROPS(17,11,0,0,"J_z");
          _SFD_SET_DATA_PROPS(18,11,0,0,"r");
          _SFD_SET_DATA_PROPS(19,11,0,0,"delta_phi");
          _SFD_SET_DATA_PROPS(20,11,0,0,"d");
          _SFD_SET_DATA_PROPS(21,11,0,0,"torque_z");
          _SFD_SET_DATA_PROPS(22,11,0,0,"d_z");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,4,0,0,0,0,0,6,3);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,4727);
        _SFD_CV_INIT_EML_IF(0,1,0,858,887,928,1173);
        _SFD_CV_INIT_EML_IF(0,1,1,933,966,1027,1169);
        _SFD_CV_INIT_EML_IF(0,1,2,1032,1060,-1,1161);
        _SFD_CV_INIT_EML_IF(0,1,3,1176,1187,1304,1340);

        {
          static int condStart[] = { 861, 873 };

          static int condEnd[] = { 869, 887 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,861,887,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 936, 953 };

          static int condEnd[] = { 949, 966 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,1,936,966,2,2,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1035, 1051 };

          static int condEnd[] = { 1047, 1060 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,2,1035,1060,2,4,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"angle_to_quat",0,-1,705);
        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"quat_conjugate",0,-1,217);
        _SFD_CV_INIT_SCRIPT(2,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(2,0,"quat_mult",0,-1,300);
        _SFD_CV_INIT_SCRIPT(3,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(3,0,"sign_l",0,-1,131);
        _SFD_CV_INIT_SCRIPT_IF(3,0,67,74,96,125);
        _SFD_CV_INIT_SCRIPT(4,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(4,0,"get_z_from_quat",0,-1,237);
        _SFD_CV_INIT_SCRIPT(5,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(5,0,"quat_inv_multiply",0,-1,337);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)
            c2_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(14,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(17,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(18,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(19,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(20,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(21,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(22,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);

        {
          real_T *c2_last_phi;
          real_T *c2_phi;
          real_T (*c2_Orientation)[3];
          real_T (*c2_Torques)[3];
          real_T (*c2_Desired_Orientation)[3];
          real_T (*c2_Angular_Vel)[3];
          c2_phi = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c2_last_phi = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c2_Angular_Vel = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S,
            2);
          c2_Desired_Orientation = (real_T (*)[3])ssGetInputPortSignal
            (chartInstance->S, 1);
          c2_Torques = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
          c2_Orientation = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S,
            0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_Orientation);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_Torques);
          _SFD_SET_DATA_VALUE_PTR(2U, *c2_Desired_Orientation);
          _SFD_SET_DATA_VALUE_PTR(3U, *c2_Angular_Vel);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_last_phi);
          _SFD_SET_DATA_VALUE_PTR(5U, c2_phi);
          _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c2_phi_low_address);
          _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c2_c_phi_address);
          _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c2_phi_up_address);
          _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c2_d_l_address);
          _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c2_v_1_address);
          _SFD_SET_DATA_VALUE_PTR(11U, chartInstance->c2_v_2_address);
          _SFD_SET_DATA_VALUE_PTR(12U, chartInstance->c2_brake_torque_address);
          _SFD_SET_DATA_VALUE_PTR(13U, chartInstance->c2_torque_xy_address);
          _SFD_SET_DATA_VALUE_PTR(14U, chartInstance->c2_t_s_address);
          _SFD_SET_DATA_VALUE_PTR(15U, chartInstance->c2_J_x_address);
          _SFD_SET_DATA_VALUE_PTR(16U, chartInstance->c2_J_y_address);
          _SFD_SET_DATA_VALUE_PTR(17U, chartInstance->c2_J_z_address);
          _SFD_SET_DATA_VALUE_PTR(18U, chartInstance->c2_r_address);
          _SFD_SET_DATA_VALUE_PTR(19U, chartInstance->c2_delta_phi_address);
          _SFD_SET_DATA_VALUE_PTR(20U, chartInstance->c2_d_address);
          _SFD_SET_DATA_VALUE_PTR(21U, chartInstance->c2_torque_z_address);
          _SFD_SET_DATA_VALUE_PTR(22U, chartInstance->c2_d_z_address);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _Thrust_Direction_quadMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "NduzaYvNQsc9gHOjC44v8F";
}

static void sf_opaque_initialize_c2_Thrust_Direction_quad(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_Thrust_Direction_quadInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_Thrust_Direction_quad
    ((SFc2_Thrust_Direction_quadInstanceStruct*) chartInstanceVar);
  initialize_c2_Thrust_Direction_quad((SFc2_Thrust_Direction_quadInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_Thrust_Direction_quad(void *chartInstanceVar)
{
  enable_c2_Thrust_Direction_quad((SFc2_Thrust_Direction_quadInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_Thrust_Direction_quad(void *chartInstanceVar)
{
  disable_c2_Thrust_Direction_quad((SFc2_Thrust_Direction_quadInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_Thrust_Direction_quad(void *chartInstanceVar)
{
  sf_c2_Thrust_Direction_quad((SFc2_Thrust_Direction_quadInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_Thrust_Direction_quad
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_Thrust_Direction_quad
    ((SFc2_Thrust_Direction_quadInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_Thrust_Direction_quad();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_Thrust_Direction_quad(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_Thrust_Direction_quad();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_Thrust_Direction_quad
    ((SFc2_Thrust_Direction_quadInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_Thrust_Direction_quad(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c2_Thrust_Direction_quad(S);
}

static void sf_opaque_set_sim_state_c2_Thrust_Direction_quad(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c2_Thrust_Direction_quad(S, st);
}

static void sf_opaque_terminate_c2_Thrust_Direction_quad(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_Thrust_Direction_quadInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_Thrust_Direction_quad_optimization_info();
    }

    finalize_c2_Thrust_Direction_quad((SFc2_Thrust_Direction_quadInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_Thrust_Direction_quad
    ((SFc2_Thrust_Direction_quadInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_Thrust_Direction_quad(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_Thrust_Direction_quad
      ((SFc2_Thrust_Direction_quadInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_Thrust_Direction_quad(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Thrust_Direction_quad_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1828051111U));
  ssSetChecksum1(S,(407112491U));
  ssSetChecksum2(S,(2486769914U));
  ssSetChecksum3(S,(1030596974U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,0);
}

static void mdlRTW_c2_Thrust_Direction_quad(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_Thrust_Direction_quad(SimStruct *S)
{
  SFc2_Thrust_Direction_quadInstanceStruct *chartInstance;
  chartInstance = (SFc2_Thrust_Direction_quadInstanceStruct *)utMalloc(sizeof
    (SFc2_Thrust_Direction_quadInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_Thrust_Direction_quadInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_Thrust_Direction_quad;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_Thrust_Direction_quad;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_Thrust_Direction_quad;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_Thrust_Direction_quad;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_Thrust_Direction_quad;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_Thrust_Direction_quad;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_Thrust_Direction_quad;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_Thrust_Direction_quad;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_Thrust_Direction_quad;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_Thrust_Direction_quad;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_Thrust_Direction_quad;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_Thrust_Direction_quad_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_Thrust_Direction_quad(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_Thrust_Direction_quad(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_Thrust_Direction_quad(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_Thrust_Direction_quad_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
