/* *******************************************************************
   **** http://www.UAVdesign.net/                                 ****
   **** To build this mex function use:                           ****
   **** mex 'cd_integrator.c'                                     ****
   ******************************************************************* */

#define S_FUNCTION_NAME cd_integrator
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NPARAMS                       14
#define paramIntegrationAlgorithm     ssGetSFcnParam(S,0)
#define paramSpecificationsSource     ssGetSFcnParam(S,1)
#define paramInitialTime              ssGetSFcnParam(S,2)
#define paramFinalTime                ssGetSFcnParam(S,3)
#define paramExternalReset            ssGetSFcnParam(S,4)
#define paramInitialConditionSource   ssGetSFcnParam(S,5)
#define paramInitialCondition         ssGetSFcnParam(S,6)
#define paramOutputTime               ssGetSFcnParam(S,7)
#define paramLimitOutput              ssGetSFcnParam(S,8)
#define paramUpperSaturationLimit     ssGetSFcnParam(S,9)
#define paramLowerSaturationLimit     ssGetSFcnParam(S,10)
#define paramQuaternionIndex          ssGetSFcnParam(S,11)
#define paramOmegaDotIndex            ssGetSFcnParam(S,12)
#define paramSampleTime               ssGetSFcnParam(S,13)

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define intval(x) ((int)((x)+0.5))

typedef real_T quaternion[4];
typedef real_T vector[3];

typedef struct
{
    int_T  nstates;
    int_T  idxin_xdot;
    int_T  idxin_x0;
    int_T  idxin_params;
    int_T  idxin_reset;
    int_T  idxout_x;
    int_T  idxout_time;
    time_T initial_time;
    int_T  nq;
    int_T  use_omegadot;
    int_T  start_idx_q;
    int_T  end_idx_q;
    int_T  start_idx_omegadot;
    int_T  end_idx_omegadot;
    vector omegadot_zero;
} CONFIG_DATA, *PCONFIG_DATA;

static void QuaternionNormalize(quaternion q);
static void QuaternionProd(const quaternion q, const quaternion r, quaternion qr_out);
static void QuaternionIntegralEulerChange(time_T delta_t, const vector qdot, const vector omegadot, const quaternion q0, quaternion phi);

/*====================*
 * S-function methods *
 *====================*/

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
/* Function: mdlCheckParameters =============================================
 * Abstract:
 *    Validate our parameters to verify they are okay.
 */
static void mdlCheckParameters(SimStruct *S)
{
    int_T param;
    
    for( param=0; param<NPARAMS; param++ )
    {
        if ( (param != 11) && (param != 12) )
        if ( !IS_PARAM_DOUBLE(ssGetSFcnParam(S, param)) && (mxGetNumberOfElements(ssGetSFcnParam(S, param)) < 1) )
        {
            ssSetErrorStatus(S, "Parameter is not a double.");
            return;
        }
    }
    
    if( (mxGetNumberOfElements(paramQuaternionIndex) != 0) && (mxGetNumberOfElements(paramQuaternionIndex) != 2) )
    {
        ssSetErrorStatus(S, "The quaternion index must be defined by two numbers.");
        return;
    }
    
    if( mxGetNumberOfElements(paramQuaternionIndex) == 2 )
    {
        if( !IS_PARAM_DOUBLE(paramQuaternionIndex) )
        {
            ssSetErrorStatus(S, "The quaternion index must be defined by of type double.");
            return;
        }
        if( (intval(mxGetPr(paramQuaternionIndex)[0]) < 1) || (intval(mxGetPr(paramQuaternionIndex)[0]) > intval(mxGetPr(paramQuaternionIndex)[1])) )
        {
            ssSetErrorStatus(S, "The quaternion index interval is not defined properly.");
            return;
        }
        
        param = intval(mxGetPr(paramQuaternionIndex)[1]) - intval(mxGetPr(paramQuaternionIndex)[0]) + 1;
        if( param != (param/4)*4 )
        {
            ssSetErrorStatus(S, "The quaternion index interval must have a length multiple of 4.");
            return;
        }
    }
    
    if( (mxGetNumberOfElements(paramOmegaDotIndex) != 0) && (mxGetNumberOfElements(paramOmegaDotIndex) != 2) )
    {
        ssSetErrorStatus(S, "The omega_dot index must be defined by two numbers.");
        return;
    }
    
    if( mxGetNumberOfElements(paramOmegaDotIndex) == 2 )
    {
        if( !IS_PARAM_DOUBLE(paramOmegaDotIndex) )
        {
            ssSetErrorStatus(S, "The omega_dot index must be defined by of type double.");
            return;
        }
        if( (intval(mxGetPr(paramOmegaDotIndex)[0]) < 1) || (intval(mxGetPr(paramOmegaDotIndex)[0]) > intval(mxGetPr(paramOmegaDotIndex)[1])) )
        {
            ssSetErrorStatus(S, "The omega_dot index interval is not defined properly.");
            return;
        }
        
        param = intval(mxGetPr(paramOmegaDotIndex)[1]) - intval(mxGetPr(paramOmegaDotIndex)[0]) + 1;
        if( param != (param/3)*3 )
        {
            ssSetErrorStatus(S, "The omega_dot index interval must have a length multiple of 3.");
            return;
        }
    }

    if ( !((mxGetPr(paramSampleTime)[0] == -1) || (mxGetPr(paramSampleTime)[0] > 0)) )
    {
        ssSetErrorStatus(S, "The sample time must be inherited (-1) or discrete (a positive number).");
        return;
    }
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
# define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{
    int_T param;
    int_T nports_in    = 1;
    int_T nports_out   = 2;
    int_T idxin_x0     = 0;
    int_T idxin_params = 0;
    int_T idxin_reset  = 0;
    int_T idxout_time  = 0;
    
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S))
    {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
	} 
    else return; /* Parameter mismatch will be reported by Simulink */
#endif

    for( param=0; param<NPARAMS; param++ )
    {
        ssSetSFcnParamTunable(S,param,false);
    }

    ssSetNumSampleTimes(S, 1);
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, DYNAMICALLY_SIZED);
    ssSetNumPWork(S, DYNAMICALLY_SIZED);
    ssSetNumDWork(S, DYNAMICALLY_SIZED);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    if( intval(mxGetScalar(paramInitialConditionSource)) > 1 )
    {
        idxin_x0 = nports_in++;
    }
    if( intval(mxGetScalar(paramSpecificationsSource)) == 3 )
    {
        idxin_params = nports_in++;
    }
    if( intval(mxGetScalar(paramExternalReset)) > 1 )
    {
        idxin_reset = nports_in++;
    }
    
    if( intval(mxGetScalar(paramOutputTime)) > 0 )
    {
        idxout_time = nports_out++;
    }
    
    if( !ssSetNumInputPorts(S, nports_in) ) return;
    if( !ssSetNumOutputPorts(S, nports_out) ) return;
    
    ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetInputPortDataType(S, 0, SS_DOUBLE );
    ssSetInputPortComplexSignal(S, 0, 0);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
    
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1, DYNAMICALLY_SIZED);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
    
    if( idxin_x0 )
    {
        ssSetInputPortWidth(S, idxin_x0, DYNAMICALLY_SIZED);
        ssSetInputPortDataType(S, idxin_x0, SS_DOUBLE );
        ssSetInputPortComplexSignal(S, idxin_x0, 0);
        ssSetInputPortDirectFeedThrough(S, idxin_x0, 1);
        ssSetInputPortRequiredContiguous(S, idxin_x0, 1); /*direct input signal access*/
    }
    if( idxin_params )
    {
        ssSetInputPortWidth(S, idxin_params, 2);
        ssSetInputPortDataType(S, idxin_params, SS_DOUBLE );
        ssSetInputPortComplexSignal(S, idxin_params, 0);
        ssSetInputPortDirectFeedThrough(S, idxin_params, 1);
    }
    if( idxin_reset )
    {
        ssSetInputPortWidth(S, idxin_reset, 1);
        ssSetInputPortDataType(S, idxin_reset, SS_BOOLEAN );
        ssSetInputPortComplexSignal(S, idxin_reset, 0);
        ssSetInputPortDirectFeedThrough(S, idxin_reset, 1);
    }
    
    if( idxout_time )
    {
        ssSetOutputPortWidth(S, idxout_time, 1);
        ssSetOutputPortDataType(S, idxout_time, SS_DOUBLE);
    }
    
    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    /* ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_CALL_TERMINATE_ON_EXIT ); */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE );
}


#if defined(MATLAB_MEX_FILE)
# define MDL_SET_INPUT_PORT_WIDTH
  static void mdlSetInputPortWidth(SimStruct *S, int_T port, int_T inputPortWidth)
  {
      if( ((ssGetInputPortWidth(S, 0) != DYNAMICALLY_SIZED) && (ssGetInputPortWidth(S, 0) != inputPortWidth)) || \
          ((ssGetOutputPortWidth(S, 1) != DYNAMICALLY_SIZED) && (ssGetOutputPortWidth(S, 1) != inputPortWidth)) )
      {
          ssSetErrorStatus(S,"xdot and x must have the same size");
          return;
      }
      if( intval(mxGetScalar(paramInitialConditionSource)) > 1 )
      {
          if( (ssGetInputPortWidth(S, 1) != DYNAMICALLY_SIZED) && (ssGetInputPortWidth(S, 1) != inputPortWidth) )
          {
              ssSetErrorStatus(S,"xdot, x0 and x must have the same size");
              return;
          }
          ssSetInputPortWidth(S, 1, inputPortWidth);
      }
      else
      {
          if( (mxGetNumberOfElements(paramInitialCondition) != 1) && (mxGetNumberOfElements(paramInitialCondition) != inputPortWidth) )
          {
              ssSetErrorStatus(S,"xdot, x0 and x must have the same size");
              return;
          }
      }
      ssSetInputPortWidth(S, 0, inputPortWidth);
      ssSetOutputPortWidth(S, 1, inputPortWidth);
  }

# define MDL_SET_OUTPUT_PORT_WIDTH
  static void mdlSetOutputPortWidth(SimStruct *S, int_T port, int_T outputPortWidth)
  {
      if( ((ssGetInputPortWidth(S, 0) != DYNAMICALLY_SIZED) && (ssGetInputPortWidth(S, 0) != outputPortWidth)) || \
          ((ssGetOutputPortWidth(S, 1) != DYNAMICALLY_SIZED) && (ssGetOutputPortWidth(S, 1) != outputPortWidth)) )
      {
          ssSetErrorStatus(S,"xdot and x must have the same size");
          return;
      }
      if( intval(mxGetScalar(paramInitialConditionSource)) > 1 )
      {
          if( (ssGetInputPortWidth(S, 1) != DYNAMICALLY_SIZED) && (ssGetInputPortWidth(S, 1) != outputPortWidth) )
          {
              ssSetErrorStatus(S,"xdot, x0 and x must have the same size");
              return;
          }
          ssSetInputPortWidth(S, 1, outputPortWidth);
      }
      else
      {
          if( (mxGetNumberOfElements(paramInitialCondition) != 1) && (mxGetNumberOfElements(paramInitialCondition) != outputPortWidth) )
          {
              ssSetErrorStatus(S,"xdot, x0 and x must have the same size");
              return;
          }
      }
      ssSetInputPortWidth(S, 0, outputPortWidth);
      ssSetOutputPortWidth(S, 1, outputPortWidth);
  }

# define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
  static void mdlSetDefaultPortDimensionInfo(SimStruct *S)
  {
      int_T size_sig;
      if( (intval(mxGetScalar(paramInitialConditionSource)) == 1) && (mxGetNumberOfElements(paramInitialCondition) > 1) )
      {
          size_sig = mxGetNumberOfElements(paramInitialCondition);
          
          if( ssGetInputPortWidth(S, 0) == DYNAMICALLY_SIZED )
          {
              ssSetInputPortWidth(S, 0, size_sig);
          }
          if( ssGetOutputPortWidth(S, 1) == DYNAMICALLY_SIZED )
          {
              ssSetOutputPortWidth(S, 1, size_sig);
          }
          if( ssGetInputPortWidth(S, 0) != ssGetOutputPortWidth(S, 1) )
          {
              ssSetErrorStatus(S,"xdot, x0 and x must have the same size");
              return;
          }
      }
  }

# define MDL_SET_WORK_WIDTHS
  static void mdlSetWorkWidths(SimStruct *S)
  {
      int_T nq = 0;
      if( mxGetNumberOfElements(paramQuaternionIndex) == 2 )
      {
          nq = intval(mxGetPr(paramQuaternionIndex)[1]) - intval(mxGetPr(paramQuaternionIndex)[0]) + 1;
          nq = nq / 4;
      }
      ssSetNumContStates(S, 0);
      ssSetNumDiscStates(S, 0);
      ssSetNumRWork(S, 0);
      ssSetNumIWork(S, 1);
      ssSetNumPWork(S, 1);
      ssSetNumDWork(S, (nq > 0 ? 3 : 2));
      ssSetDWorkWidth(S, 0, ssGetInputPortWidth(S, 0));
      ssSetDWorkWidth(S, 1, ssGetInputPortWidth(S, 0));
      ssSetDWorkDataType(S, 0, SS_DOUBLE);
      ssSetDWorkDataType(S, 1, SS_DOUBLE);
      if( nq )
      {
          ssSetDWorkWidth(S, 2, ssGetInputPortWidth(S, 0));
          ssSetDWorkDataType(S, 2, SS_DOUBLE);
      }
      ssSetNumModes(S, 0);
  }
#endif


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, *mxGetPr(paramSampleTime));
	ssSetOffsetTime(S, 0, 0.0);
    ssSetCallSystemOutput(S,0);  /* call on first element */
	ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ==========================================
 * Abstract:
 *      Initalize the states.
 */
static void mdlInitializeConditions(SimStruct *S)
{
    /* Reset the IWork flag to 1 when values need to be reinitialized.*/
    ssSetIWorkValue(S, 0, 1);
} /* end mdlInitializeConditions */



/* Function: mdlStart =======================================================
 * Abstract:
 * This function is called once at start of model execution. If you
 * have states that should be initialized once, this is the place
 * to do it.
 */
#define MDL_START
static void mdlStart(SimStruct *S)
{
    PCONFIG_DATA config;
    int_T nports_in    = 1;
    int_T nports_out   = 2;
    int_T nq;
    
    /* Store new C object in the pointers vector */
    config = (PCONFIG_DATA) calloc(1, sizeof(CONFIG_DATA));
    ssGetPWork(S)[0] = config;
    
    /* Store the number of ports and their indices */
    config->idxin_xdot = 0;
    if( intval(mxGetScalar(paramInitialConditionSource)) > 1 )
    {
        config->idxin_x0 = nports_in++;
    }
    else
    {
        config->idxin_x0 = 0;
    }
    if( intval(mxGetScalar(paramSpecificationsSource)) == 3 )
    {
        config->idxin_params = nports_in++;
    }
    else
    {
        config->idxin_params = 0;
    }
    if( intval(mxGetScalar(paramExternalReset)) > 1 )
    {
        config->idxin_reset = nports_in++;
    }
    else
    {
        config->idxin_reset = 0;
    }
    
    config->idxout_x = 1;
    if( intval(mxGetScalar(paramOutputTime)) > 0 )
    {
        config->idxout_time = nports_out++;
    }
    else
    {
        config->idxout_time = 0;
    }
    
    /* Store the number of quaternions */
    if( mxGetNumberOfElements(paramQuaternionIndex) == 2 )
    {
        config->start_idx_q = intval(mxGetPr(paramQuaternionIndex)[0]) - 1;
        config->end_idx_q = intval(mxGetPr(paramQuaternionIndex)[1]) - 1;
        nq = 1 + config->end_idx_q - config->start_idx_q;
        nq = nq / 4;
        config->nq = nq;
        if( mxGetNumberOfElements(paramOmegaDotIndex) == 2 )
        {
            config->start_idx_omegadot = intval(mxGetPr(paramOmegaDotIndex)[0]) - 1;
            config->end_idx_omegadot = intval(mxGetPr(paramOmegaDotIndex)[1]) - 1;
            config->use_omegadot = 1;
        }
    }
    
    config->initial_time = ssGetTStart(S);
    config->nstates = ssGetInputPortWidth(S, 0);
}


/* Function: mdlOutputs =======================================================
 *
 */
#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
{
    PCONFIG_DATA config;
    const real_T         *in_xdot;
    const real_T         *in_x0;
    InputRealPtrsType    pin_params;
    const boolean_T      *in_reset;
    real_T               *out_x;
    real_T               *out_t;
    real_T               *xd;
    real_T               *xd_temp;
    real_T               *xd_temp2;
    time_T               initial_time;
    time_T               final_time;
    quaternion           phi;
    quaternion           q;
    vector               omegadot_temp;
    const real_T         *pomegadot;
    int_T                i;

    /* Retrieve C object from the pointers vector */
    config = ssGetPWork(S)[0];

    xd = (real_T*) ssGetDWork(S,0);
    xd_temp = (real_T*) ssGetDWork(S,1);
    if( config->nq ) xd_temp2 = (real_T*) ssGetDWork(S,2);
    in_xdot = ssGetInputPortRealSignal(S, config->idxin_xdot);
    if( config->idxin_x0 ) in_x0 = ssGetInputPortRealSignal(S, config->idxin_x0);
    if( config->idxin_params ) pin_params = ssGetInputPortRealSignalPtrs(S, config->idxin_params);
    if( config->idxin_reset ) in_reset = ((InputBooleanPtrsType) ssGetInputPortSignalPtrs(S, config->idxin_reset))[0];
    out_x = ssGetOutputPortRealSignal(S, 1);
    if( config->idxout_time ) out_t = ssGetOutputPortRealSignal(S, config->idxout_time);

    switch( intval(mxGetScalar(paramSpecificationsSource)) )
    {
        case 1:
            initial_time = config->initial_time;
            final_time   = ssGetTaskTime(S,0);
            break;
        case 2:
            initial_time = mxGetScalar(paramInitialTime);
            final_time   = mxGetScalar(paramFinalTime);
            break;
        case 3:
            initial_time = *(pin_params[0]);
            final_time   = *(pin_params[1]);
            break;
        default:
            ssSetErrorStatus(S,"Wrong integration algorithm selected");
            return;
    }
    
/*    ssPrintf("ti=%f, tf=%f\r\n", initial_time, final_time); */

    /* Reset the states */
    if( ssGetIWorkValue(S, 0) || (config->idxin_reset && *in_reset) || (intval(mxGetScalar(paramSpecificationsSource)) > 1) )
    {
        ssSetIWorkValue(S, 0, 0);
        if( intval(mxGetScalar(paramInitialConditionSource)) == 1 )
        {
            /* Internal initial conditions */
            for( i=0; i<config->nstates; i++ )
            {
                xd[i] = mxGetPr(paramInitialCondition)[(mxGetNumberOfElements(paramInitialCondition) == 1 ? 0 : i)];
            }
        }
        else
        {
            /* External initial conditions */
            memcpy(xd, in_x0, config->nstates*sizeof(real_T));
        }
        memcpy(out_x, xd, config->nstates*sizeof(real_T));
        if( config->idxout_time ) out_t[0] = initial_time;
    }
    
    if( final_time > initial_time )
    {
        if( intval(mxGetScalar(paramIntegrationAlgorithm)) == 1 )
        {
            /* Euler algorithm */
            if( !ssCallSystemWithTid(S,0,tid) ) return;
            i = 0;
            while( i<config->nstates )
            {
                if( config->nq && (i >= config->start_idx_q) && (i < config->end_idx_q) )
                {
                    pomegadot = ( config->use_omegadot ? &in_xdot[i] : config->omegadot_zero );
                    QuaternionIntegralEulerChange( final_time-initial_time, &in_xdot[i], pomegadot, &xd[i], phi );
                    QuaternionProd(&xd[i], phi, &xd[i]);
                    QuaternionNormalize(&xd[i]);
                    i += 4;
                }
                else
                {
                    xd[i] += in_xdot[i] * (final_time-initial_time);
                    i++;
                }
            }
        }
        
        if( intval(mxGetScalar(paramIntegrationAlgorithm)) == 2 )
        {
            /* Runge-Kutta algorithm */
            /* f1 */
            if( !ssCallSystemWithTid(S,0,tid) ) return;
            i = 0;
            while( i<config->nstates )
            {
                if( config->nq && (i >= config->start_idx_q) && (i < config->end_idx_q) )
                {
                    pomegadot = ( config->use_omegadot ? &in_xdot[i] : config->omegadot_zero );
                    omegadot_temp[0] = pomegadot[0]*6; omegadot_temp[1] = pomegadot[1]*6; omegadot_temp[2] = pomegadot[2]*6;
                    QuaternionIntegralEulerChange( (final_time-initial_time)/6, &in_xdot[i], omegadot_temp, &xd[i], &xd_temp[i] );
                    QuaternionProd(&xd_temp[i], &xd_temp[i], q);
                    QuaternionProd(&xd_temp[i], q, phi);
                    QuaternionProd(&xd[i], phi, &out_x[i]);
                    i += 4;
                }
                else
                {
                    xd_temp[i] = in_xdot[i];
                    out_x[i] = xd[i] + 0.5*(final_time-initial_time)*in_xdot[i];
                    i++;
                }
            }
            if( config->idxout_time ) out_t[0] = initial_time + 0.5*(final_time-initial_time);
            
            /* f2 */
            if( !ssCallSystemWithTid(S,0,tid) ) return;
            i = 0;
            while( i<config->nstates )
            {
                if( config->nq && (i >= config->start_idx_q) && (i < config->end_idx_q) )
                {
                    pomegadot = ( config->use_omegadot ? &in_xdot[i] : config->omegadot_zero );
                    omegadot_temp[0] = pomegadot[0]*6; omegadot_temp[1] = pomegadot[1]*6; omegadot_temp[2] = pomegadot[2]*6;
                    QuaternionIntegralEulerChange( (final_time-initial_time)/6, &in_xdot[i], omegadot_temp, &xd[i], q );
                    QuaternionProd(q, q, phi);
                    QuaternionProd(&xd_temp[i], phi, &xd_temp[i]);
                    QuaternionProd(phi, q, phi);
                    QuaternionProd(&xd[i], phi, &out_x[i]);
                    i += 4;
                }
                else
                {
                    xd_temp[i] += 2*in_xdot[i];
                    out_x[i] = xd[i] + 0.5*(final_time-initial_time)*in_xdot[i];
                    i++;
                }
            }
            if( config->idxout_time ) out_t[0] = initial_time + 0.5*(final_time-initial_time);
            
            /* f3 */
            if( !ssCallSystemWithTid(S,0,tid) ) return;
            i = 0;
            while( i<config->nstates )
            {
                if( config->nq && (i >= config->start_idx_q) && (i < config->end_idx_q) )
                {
                    pomegadot = ( config->use_omegadot ? &in_xdot[i] : config->omegadot_zero );
                    omegadot_temp[0] = pomegadot[0]*6; omegadot_temp[1] = pomegadot[1]*6; omegadot_temp[2] = pomegadot[2]*6;
                    QuaternionIntegralEulerChange( (final_time-initial_time)/6, &in_xdot[i], omegadot_temp, &xd[i], q );
                    QuaternionProd(q, q, phi);
                    QuaternionProd(&xd_temp[i], phi, &xd_temp[i]);
                    QuaternionProd(phi, q, phi);
                    QuaternionProd(phi, phi, phi);
                    QuaternionProd(&xd[i], phi, &out_x[i]);
                    i += 4;
                }
                else
                {
                    xd_temp[i] += 2*in_xdot[i];
                    out_x[i] = xd[i] + (final_time-initial_time)*in_xdot[i];
                    i++;
                }
            }
            if( config->idxout_time ) out_t[0] = final_time;
            
            /* f4 */
            if( !ssCallSystemWithTid(S,0,tid) ) return;
            i = 0;
            while( i<config->nstates )
            {
                if( config->nq && (i >= config->start_idx_q) && (i < config->end_idx_q) )
                {
                    pomegadot = ( config->use_omegadot ? &in_xdot[i] : config->omegadot_zero );
                    omegadot_temp[0] = pomegadot[0]*6; omegadot_temp[1] = pomegadot[1]*6; omegadot_temp[2] = pomegadot[2]*6;
                    QuaternionIntegralEulerChange( (final_time-initial_time)/6, &in_xdot[i], omegadot_temp, &xd[i], q );
                    QuaternionProd(&xd_temp[i], q, &xd_temp[i]);
                    QuaternionProd(&xd[i], &xd_temp[i], &xd[i]);
                    QuaternionNormalize(&xd[i]);
                    i += 4;
                }
                else
                {
                    xd_temp[i] += in_xdot[i];
                    xd[i] += 1.0/6*(final_time-initial_time)*xd_temp[i];
                    i++;
                }
            }
        }
    }
    else
    {
        if( !ssCallSystemWithTid(S,0,tid) ) return;
    }

    config->initial_time = final_time;

    /* Update the outputs */
    memcpy(out_x, xd, config->nstates*sizeof(real_T));
    if( config->idxout_time ) out_t[0] = final_time;
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
#define MDL_TERMINATE
static void mdlTerminate(SimStruct *S)
{
    PCONFIG_DATA config;

    /* Retrieve and destroy C object */
    if( ssGetNumPWork(S) > 0 )
    {
        config = ssGetPWork(S)[0];
        if( config )
        {
            free(config);
            ssGetPWork(S)[0] = 0;
        }
    }
}


static void QuaternionNormalize(quaternion q)
{
    int_T j;
    real_T norm;
    
    norm = 1/(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for( j=0; j<4; j++ )
    {
        q[j] = q[j]*norm;
    }
}

static void QuaternionProd(const quaternion q, const quaternion r, quaternion qr_out)
{
    quaternion q_int;
    
    q_int[0] = q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3];
    q_int[1] = q[1]*r[0] + q[0]*r[1] - q[3]*r[2] + q[2]*r[3];
    q_int[2] = q[2]*r[0] + q[3]*r[1] + q[0]*r[2] - q[1]*r[3];
    q_int[3] = q[3]*r[0] - q[2]*r[1] + q[1]*r[2] + q[0]*r[3];
    
    qr_out[0] = q_int[0];
    qr_out[1] = q_int[1];
    qr_out[2] = q_int[2];
    qr_out[3] = q_int[3];
}

static void QuaternionIntegralEulerChange( time_T delta_t, const vector qdot, const vector omegadot, const quaternion q0, quaternion phi )
{
    int_T         j;
    vector        delta;
    real_T        delta_norm_sq;
    quaternion    omega0;
    quaternion    omega1;
    quaternion    omega0_omega1;
    quaternion    omega1_omega0;
    quaternion    q;
    real_T        delta_t3;
    int_T         squaring_times;
    int_T         k;
    real_T        max_elem;

    /* Calculate omega from qdot */
    q[0] = 2 * q0[0];
    for( j=1; j<4; j++ )
    {
        q[j] = - 2 * q0[j];
    }
    QuaternionProd(q, qdot, omega0);
    
    /* Calculate delta as Euler integral of (omega / 2) */
    for( j=0; j<3; j++ )
    {
        delta[j] = 0.5*omega0[j+1]*delta_t;
    }
        
    /* quaternion transition matrix */
    /* using repeated squaring to improve precision */
    max_elem = abs(delta[0]);
    for( j=1; j<3; j++ )
    {
        if( abs(delta[j]) > max_elem ) max_elem = abs(delta[j]);
    }
    squaring_times = max_elem / 0.25;
    
    for(k = 0; k < squaring_times; k++ )
    for( j=0; j<3; j++ )
    {
        delta[j] = delta[j] / 2;
    }
    delta_norm_sq = 0;
    for( j=0; j<3; j++ )
    {
        delta_norm_sq += delta[j] * delta[j];
    }
    
    /* third order Taylor approximation */
    phi[0] = 1 - 0.5 * delta_norm_sq;
    for( j=0; j<3; j++ )
    {
        phi[j+1] = delta[j] * (1 - 1/6*delta_norm_sq);
    }
    
    for( k = 0; k < squaring_times; k++ )
    {
        QuaternionProd(phi, phi, q);
        for( j=0; j<4; j++ )
        {
            phi[j] = q[j];
        }
    }
    
    /* coning motion error term */
    omega1[0] = 0;
    for( j=0; j<3; j++ )
    {
        omega1[j+1] = omegadot[j];
    }
    QuaternionProd(omega0, omega1, omega0_omega1);
    QuaternionProd(omega1, omega0, omega1_omega0);

    delta_t3 = delta_t * delta_t * delta_t;
    for( j=0; j<4; j++ )
    {
        phi[j] += 1/48*(omega0_omega1[j]-omega1_omega0[j])*delta_t3;
    }
    
    /* normalize */
    /* delta_t3 = 1/(phi[0]*phi[0] + phi[1]*phi[1] + phi[2]*phi[2] + phi[3]*phi[3]);
       for( j=0; j<4; j++ )
       {
           phi[j] = phi[j]*delta_t3;
       }
     */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
