#ifndef __c2_Thrust_Direction_quad_h__
#define __c2_Thrust_Direction_quad_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_c2_ResolvedFunctionInfo
#define typedef_c2_ResolvedFunctionInfo

typedef struct {
  const char * context;
  const char * name;
  const char * dominantType;
  const char * resolved;
  uint32_T fileTimeLo;
  uint32_T fileTimeHi;
  uint32_T mFileTimeLo;
  uint32_T mFileTimeHi;
} c2_ResolvedFunctionInfo;

#endif                                 /*typedef_c2_ResolvedFunctionInfo*/

#ifndef typedef_SFc2_Thrust_Direction_quadInstanceStruct
#define typedef_SFc2_Thrust_Direction_quadInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_Thrust_Direction_quad;
  real_T *c2_J_x_address;
  int32_T c2_J_x_index;
  real_T *c2_J_y_address;
  int32_T c2_J_y_index;
  real_T *c2_J_z_address;
  int32_T c2_J_z_index;
  real_T *c2_brake_torque_address;
  int32_T c2_brake_torque_index;
  real_T *c2_c_phi_address;
  int32_T c2_c_phi_index;
  real_T *c2_d_address;
  int32_T c2_d_index;
  real_T *c2_d_l_address;
  int32_T c2_d_l_index;
  real_T *c2_d_z_address;
  int32_T c2_d_z_index;
  real_T *c2_delta_phi_address;
  int32_T c2_delta_phi_index;
  real_T *c2_phi_low_address;
  int32_T c2_phi_low_index;
  real_T *c2_phi_up_address;
  int32_T c2_phi_up_index;
  real_T *c2_r_address;
  int32_T c2_r_index;
  real_T *c2_t_s_address;
  int32_T c2_t_s_index;
  real_T *c2_torque_xy_address;
  int32_T c2_torque_xy_index;
  real_T *c2_torque_z_address;
  int32_T c2_torque_z_index;
  real_T *c2_v_1_address;
  int32_T c2_v_1_index;
  real_T *c2_v_2_address;
  int32_T c2_v_2_index;
} SFc2_Thrust_Direction_quadInstanceStruct;

#endif                                 /*typedef_SFc2_Thrust_Direction_quadInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c2_Thrust_Direction_quad_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_Thrust_Direction_quad_get_check_sum(mxArray *plhs[]);
extern void c2_Thrust_Direction_quad_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
