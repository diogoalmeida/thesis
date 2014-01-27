/*
 *      http://www.UAVdesign.net/
 *      Integrates quaternions
 *      Compile with: mex 'quaternionintegrator.c'
 */

#define S_FUNCTION_NAME  quaternionintegrator
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define InitialConditionExternal(S)  ssGetSFcnParam(S, 0)
#define InitialCondition(S)          ssGetSFcnParam(S, 1)
#define RotationAxisChange(S)        ssGetSFcnParam(S, 2)
#define NPARAMS 3
#define MAX_ITEMS 10


typedef real_T quaternion[4];
typedef real_T vector[3];

typedef struct 
{
    quaternion          q0;
    quaternion          q_dot;
    vector              omega_dot;
    vector              omega_int;
} quaternion_item;

typedef quaternion_item quaternion_history[MAX_ITEMS];

typedef struct
{
    int_T               q_count;
    int_T               item_count;
    quaternion_history* q_list;
    time_T              t0[MAX_ITEMS];
    int_T               index_omega_ddot;
    int_T               index_q0_ext;
} WORK_DATA;

static void QuaternionProd(const quaternion q, const quaternion r, quaternion qr_out);
static void QuaternionIntegral(time_T delta_t, const quaternion_item* q0_item, const vector omega_int, quaternion q_int );

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
  /* Function: mdlCheckParameters =============================================
   * Abstract:
   *    Validate our parameters to verify:
   */
static void mdlCheckParameters(SimStruct *S)
{
    real_T *pr;
    int_T   el;
    int_T   nEls;
    
    if (mxIsEmpty(    InitialConditionExternal(S)) ||
      mxIsSparse(   InitialConditionExternal(S)) ||
      mxIsComplex(  InitialConditionExternal(S)) ||
      mxIsLogical(  InitialConditionExternal(S)) ||
      !mxIsNumeric( InitialConditionExternal(S)) || 
      !mxIsDouble(   InitialConditionExternal(S)) )
    {
      ssSetErrorStatus(S,"The initial condition type has to be 0 or 1.");
      return;
    }
    
    pr   = mxGetPr(InitialConditionExternal(S));
    nEls = mxGetNumberOfElements(InitialConditionExternal(S));
    
    if ((nEls != 1) || ((pr[0] != 0) && (pr[0] != 1)))
    {
      ssSetErrorStatus(S,"The initial condition type has to be 0 or 1.");
      return;
    }
    
    
    if (mxIsEmpty(  RotationAxisChange(S)) ||
      mxIsSparse(   RotationAxisChange(S)) ||
      mxIsComplex(  RotationAxisChange(S)) ||
      mxIsLogical(  RotationAxisChange(S)) ||
      !mxIsNumeric( RotationAxisChange(S)) || 
      !mxIsDouble(  RotationAxisChange(S)) )
    {
      ssSetErrorStatus(S,"The rotation axis changing type has to be 0 or 1.");
      return;
    }
    
    pr   = mxGetPr(RotationAxisChange(S));
    nEls = mxGetNumberOfElements(RotationAxisChange(S));
    
    if ((nEls != 1) || ((pr[0] != 0) && (pr[0] != 1)))
    {
      ssSetErrorStatus(S,"The rotation axis changing type has to be 0 or 1.");
      return;
    }
    

    if( pr[0] == 0 )
    {
        if (mxIsEmpty(    InitialCondition(S)) ||
          mxIsSparse(   InitialCondition(S)) ||
          mxIsComplex(  InitialCondition(S)) ||
          mxIsLogical(  InitialCondition(S)) ||
          !mxIsNumeric( InitialCondition(S)) || 
          !mxIsDouble(   InitialCondition(S)) ) 
        {
          ssSetErrorStatus(S,"The initial state must be a real finite vector");
          return;
        } 
        pr   = mxGetPr(InitialCondition(S));
        nEls = mxGetNumberOfElements(InitialCondition(S));
        for (el = 0; el < nEls; el++)
        {
          if (!mxIsFinite(pr[el]))
          {
              ssSetErrorStatus(S,"The initial state must be a real finite vector");
              return;
          }
        }

        if ((mxGetNumberOfElements(InitialCondition(S)) % 4) != 0)
        {
          ssSetErrorStatus(S,"The initial condition represents one or more quaternions. It has to be a multiple of 4 in size.");
          return;
        }
    }
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    const real_T   *pr;
    int_T    q_count = 1;
    int_T param;
    
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters. */
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return; /* Parameter mismatch will be reported by Simulink. */
    }
#endif

    for( param=0; param<NPARAMS; param++ )
    {
        ssSetSFcnParamTunable(S,param,false);
    }

    ssSetNumContStates(S, q_count * 3);
    ssSetNumDiscStates(S, 0);

    pr   = mxGetPr(InitialConditionExternal(S));
    if (pr[0] > 0)
    {
        pr   = mxGetPr(RotationAxisChange(S));
        if (pr[0] > 0)
        {
            if (!ssSetNumInputPorts(S, 3)) return;
            /* omega_dot */
            ssSetInputPortWidth(S, 1, q_count * 3);
            ssSetInputPortDirectFeedThrough(S, 1, 0);
            /* q_0 */
            ssSetInputPortWidth(S, 2, q_count * 4);
            ssSetInputPortDirectFeedThrough(S, 2, 1);
        }
        else
        {
            if (!ssSetNumInputPorts(S, 2)) return;
            /* q_0 */
            ssSetInputPortWidth(S, 1, q_count * 4);
            ssSetInputPortDirectFeedThrough(S, 1, 1);
        }
    }
    else
    {
        pr   = mxGetPr(RotationAxisChange(S));
        if (pr[0] > 0)
        {
            if (!ssSetNumInputPorts(S, 2)) return;
            /* omega_dot */
            ssSetInputPortWidth(S, 1, q_count * 3);
            ssSetInputPortDirectFeedThrough(S, 1, 0);
        }
        else
        {
            if (!ssSetNumInputPorts(S, 1)) return;
        }
    }
    
    /* q_dot */
    ssSetInputPortWidth(S, 0, q_count * 4);
    ssSetInputPortDirectFeedThrough(S, 0, 0);

    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, q_count * 4);

    ssSetNumSampleTimes(S, 1);

    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 1);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE));

} /* end mdlInitializeSizes */


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *      This function is used to specify the sample time(s) for the
 *      S-function.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);

} /* end mdlInitializeSampleTimes */



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



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
/* Function: mdlStart =======================================================
* Abstract:
*    This function is called once at start of model execution. If you
*    have states that should be initialized once, this is the place
*    to do it.
*/
static void mdlStart(SimStruct *S)
{
    const real_T *pr;
    WORK_DATA* wd = (WORK_DATA *) malloc(sizeof(WORK_DATA));
    if (wd == NULL)
    {
        ssSetErrorStatus(S, "Memory allocation error.");
        return;
    }
    ssSetUserData(S, wd);
    
    wd->q_list  = NULL;
    wd->q_count = ssGetInputPortWidth(S, 0) / 4;
    wd->q_list  = (quaternion_history *) malloc(wd->q_count * sizeof(quaternion_history));
    wd->index_omega_ddot = 0;
    wd->index_q0_ext = 0;
    
    pr   = mxGetPr(InitialConditionExternal(S));
    if( pr[0] > 0 )
    {
        pr   = mxGetPr(RotationAxisChange(S));
        if( pr[0] > 0 )
        {
            wd->index_omega_ddot = 1;
            wd->index_q0_ext = 2;
        }
        else
        {
            wd->index_q0_ext = 1;
        }
    }
    else
    {
        pr   = mxGetPr(RotationAxisChange(S));
        if( pr[0] > 0 )
        {
            wd->index_omega_ddot = 1;
        }
    }
}
#endif /*  MDL_START */


/* Function: mdlOutputs =======================================================
 * Abstract:
 *      The outputs for this block are computed by using a controllable state-
 *      space representation of the transfer function.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    int_T             i;
    int_T             j;
    const real_T      *q0_int;
    time_T            t           = ssGetT(S);
    int_T             most_recent;
    WORK_DATA         *wd         = (WORK_DATA *)ssGetUserData(S);
    InputRealPtrsType uPtrs       = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType uPtrs_q0;
    real_T            *y          = ssGetOutputPortRealSignal(S,0);
    real_T            *omega_int  = ssGetContStates(S);
    quaternion        q_current;
    
    /* Initialize values if the IWork vector flag is true. */
    if (ssGetIWorkValue(S, 0) == 1)
    {
        /* Enter initialization code here */
        real_T *x0 = ssGetContStates(S);
        x0[0] = 0;
        ssSetIWorkValue(S, 0, 0);

        if( wd->index_q0_ext )
        {
            uPtrs_q0 = ssGetInputPortRealSignalPtrs(S,wd->index_q0_ext);
        }
        else
        {
            q0_int = mxGetPr(InitialCondition(S));
        }
        for( i = 0; i < wd->q_count; i++ )
        {
            wd->item_count = 1;
            wd->t0[0] = t;
            for( j = 0; j < 4; j++ )
            {
                wd->q_list[i][0].q0[j] = (wd->index_q0_ext ? *uPtrs_q0[i*4+j] : q0_int[i*4+j]);
                if(j < 3) wd->q_list[i][0].omega_int[j] = 0;
            }
        }
    }
    
    /* Search for the most recent sample time in the history */
    if( t < wd->t0[0] )
    {
        ssPrintf("Warning: Output function called with a time lower than the previous major step time.\nPrevious t=%f, current t=%f\r\n", wd->t0[0], t);
        most_recent = 0;
        /* ssSetErrorStatus(S, "Output function called with a time lower than the previous major step time.");
        return; */
    }
    
    if( t <= wd->t0[0] )
    {
        most_recent = 0;
    }
    else if( t >= wd->t0[wd->item_count-1] )
    {
        /* most recent is at the end */
        most_recent = wd->item_count-1;
    }
    else
    {
        /* search for a position in the middle */
        most_recent = 0;
        while( t >= wd->t0[most_recent+1] )
        {
            most_recent++;
        }
    }
    
    /* Set the output values */
    for( i = 0; i < wd->q_count; i++ )
    {
        /* calculate q for the current step */
        QuaternionIntegral( t - wd->t0[most_recent], &wd->q_list[i][most_recent], &omega_int[i*3], q_current );
        for( j = 0; j < 4; j++ )
        {
            y[i*4+j] = q_current[j];
        }
    }

    if (ssIsMajorTimeStep(S))
    {
        /* Keep only the most recent entry from the item list */
        if( wd->item_count > 1 )
        {
            for( i = 0; i < wd->q_count; i++ )
            {
                for( j = 0; j < 4; j++ )
                {
                    wd->q_list[i][0].q0[j] = wd->q_list[i][most_recent].q0[j];
                    wd->q_list[i][0].q_dot[j] = wd->q_list[i][most_recent].q_dot[j];
                    if(j<3)
                    {
                        wd->q_list[i][0].omega_dot[j] = wd->q_list[i][most_recent].omega_dot[j];
                        wd->q_list[i][0].omega_int[j] = wd->q_list[i][most_recent].omega_int[j];
                    }
                }
            }
            wd->t0[0] = wd->t0[most_recent];
            wd->item_count = 1;
        }
    }

} /* end mdlOutputs */


#define MDL_DERIVATIVES
/* Function: mdlDerivatives ===================================================
 */
static void mdlDerivatives(SimStruct *S) 
{
    WORK_DATA         *wd         = (WORK_DATA *)ssGetUserData(S);
    real_T            *omega      = ssGetdX(S);
    real_T            *omega_int  = ssGetContStates(S);
    InputRealPtrsType q_dotPtrs;
    InputRealPtrsType omega_ddotPtrs;
    InputRealPtrsType q0Ptrs;
    const real_T      *q0_int;
    time_T            t           = ssGetT(S);
    int_T             i;
    int_T             j;
    int_T             index;
    int_T             pos_item;
    quaternion        q0_current;
    quaternion        omega_q;
    
    /* Get the pointers to the inputs */
    q_dotPtrs = ssGetInputPortRealSignalPtrs(S,0);
    if( wd->index_omega_ddot )
    {
        omega_ddotPtrs = ssGetInputPortRealSignalPtrs(S,wd->index_omega_ddot);
    }
    if( wd->index_q0_ext )
    {
        q0Ptrs = ssGetInputPortRealSignalPtrs(S,wd->index_q0_ext);
    }
    else
    {
        q0_int = mxGetPr(InitialCondition(S));
    }
    
    if( t < wd->t0[0] )
    {
        ssPrintf("Warning: The current time value is smaller than at the beginning of the major time step.\r\n");
    }
    
    if( wd->item_count == MAX_ITEMS )
    {
        ssPrintf("Warning: Maximum number of items reached in minor time step for quaternion integration.\r\n");
    }
    
    
    if( t <= wd->t0[0] )
    {
        pos_item = 0;
    } 
    else if( t > wd->t0[wd->item_count-1] )
    {
        /* insert at the end */
        pos_item = (wd->item_count < MAX_ITEMS ? wd->item_count : wd->item_count - 1);
    }
    else
    {
        /* search for a position in the middle where to insert */
        for( index = 1; ; index++ )
        {
            if( t <= wd->t0[index] )
            {
                pos_item = index;
                break;
            }
        }
    }

    if( (t > wd->t0[0]) && (wd->item_count < MAX_ITEMS) )
    if( (pos_item == wd->item_count) || (t != wd->t0[pos_item]) )
    {
        for( i = 0; i < wd->q_count; i++ )
        {
            /* calculate q0 for the current step */
            QuaternionIntegral(t - wd->t0[pos_item-1], &wd->q_list[i][pos_item-1], &omega_int[i*3], q0_current );
            /* shift the data to make space for the inserted values */
            for( index = wd->item_count; index > pos_item; index-- )
            {
                for( j = 0; j < 4; j++ )
                {
                    wd->q_list[i][index].q0[j] = wd->q_list[i][index-1].q0[j];
                    wd->q_list[i][index].q_dot[j] = wd->q_list[i][index-1].q_dot[j];
                    if(j<3)
                    {
                        wd->q_list[i][index].omega_dot[j] = wd->q_list[i][index-1].omega_dot[j];
                        wd->q_list[i][index].omega_int[j] = wd->q_list[i][index-1].omega_int[j];
                    }
                }
            }
            /* insert the values */
            for( j = 0; j < 4; j++ )
            {
                wd->q_list[i][pos_item].q0[j] = q0_current[j];
                wd->q_list[i][pos_item].q_dot[j] = *q_dotPtrs[i*4+j];
                if(j<3)
                {
                    wd->q_list[i][pos_item].omega_dot[j] = (wd->index_omega_ddot ? *omega_ddotPtrs[i*3+j] : 0);
                    wd->q_list[i][pos_item].omega_int[j] = omega_int[i*3+j];
                }
            }
        }
        for( index = wd->item_count; index > pos_item; index-- )
        {
            wd->t0[index] = wd->t0[index-1];
        }
        wd->t0[pos_item] = t;
        wd->item_count++;
    }
    
    /* Set the state derivatives (omega) */
    for( i = 0; i < wd->q_count; i++ )
    {
        for( j=0; j<4; j++ )
        {
            q0_current[j] = 2 * wd->q_list[i][pos_item].q0[j];
            /* conjugate */
            if(j) q0_current[j] = -q0_current[j];
        }
        QuaternionProd(q0_current, wd->q_list[i][pos_item].q_dot, omega_q);
        for( j=0; j<3; j++ )
        {
            omega[i*3+j] = omega_q[j+1];
        }
    }

} /* end mdlDerivatives */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *      Called when the simulation is terminated.
 *      For this block, there are no end of simulation tasks.
 */
static void mdlTerminate(SimStruct *S)
{
    WORK_DATA *wd = (WORK_DATA *)ssGetUserData(S);
    if (wd != NULL)
    {
        if (wd->q_list != NULL)
        {
            free(wd->q_list);
        }
        free(wd);
    }
} /* end mdlTerminate */


void QuaternionProd(const quaternion q, const quaternion r, quaternion qr_out)
{
    qr_out[0] = q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3];
    qr_out[1] = q[1]*r[0] + q[0]*r[1] - q[3]*r[2] + q[2]*r[3];
    qr_out[2] = q[2]*r[0] + q[3]*r[1] + q[0]*r[2] - q[1]*r[3];
    qr_out[3] = q[3]*r[0] - q[2]*r[1] + q[1]*r[2] + q[0]*r[3];
}

void QuaternionIntegral( time_T delta_t, const quaternion_item* q0_item, const vector omega_int, quaternion q_int )
{
    int_T         j;
    vector        delta_omega_int;
    real_T        delta_omega_int_norm_sq;
    quaternion    phi;
    quaternion    omega0;
    quaternion    omega1;
    quaternion    omega0_omega1;
    quaternion    omega1_omega0;
    quaternion    q;
    real_T        delta_t3;
    int_T         squaring_times;
    int_T         k;
    real_T        max_elem;

    for( j=0; j<3; j++ )
    {
        delta_omega_int[j] = (omega_int[j] - q0_item->omega_int[j]) / 2;
    }
        
    /* quaternion transition matrix */
    /* using repeated squaring to improve precision */
    max_elem = abs(delta_omega_int[0]);
    for( j=1; j<3; j++ )
    {
        if( abs(delta_omega_int[j]) > max_elem ) max_elem = abs(delta_omega_int[j]);
    }
    squaring_times = max_elem / 0.25;
    
    for(k = 0; k < squaring_times; k++ )
    for( j=0; j<3; j++ )
    {
        delta_omega_int[j] = delta_omega_int[j] / 2;
    }
    delta_omega_int_norm_sq = 0;
    for( j=0; j<3; j++ )
    {
        delta_omega_int_norm_sq += delta_omega_int[j] * delta_omega_int[j];
    }
    
    /* third order Taylor approximation */
    phi[0] = 1 - 0.5 * delta_omega_int_norm_sq;
    for( j=0; j<3; j++ )
    {
        phi[j+1] = delta_omega_int[j] * (1 - 1/6*delta_omega_int_norm_sq);
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
    for( j=0; j<4; j++ )
    {
        q[j] = 2 * q0_item->q0[j];
        if(j) q[j] = -q[j];
    }
    QuaternionProd(q, q0_item->q_dot, omega0);
    omega1[0] = 0;
    for( j=0; j<3; j++ )
    {
        omega1[j+1] = q0_item->omega_dot[j];
    }
    QuaternionProd(omega0, omega1, omega0_omega1);
    QuaternionProd(omega1, omega0, omega1_omega0);

    delta_t3 = delta_t * delta_t * delta_t;
    for( j=0; j<4; j++ )
    {
        phi[j] += 1/48*(omega0_omega1[j]-omega1_omega0[j])*delta_t3;
    }

    QuaternionProd(q0_item->q0, phi, q_int);
    
    /* normalize */
    delta_t3 = 1/(q_int[0]*q_int[0] + q_int[1]*q_int[1] + q_int[2]*q_int[2] + q_int[3]*q_int[3]);
    for( j=0; j<4; j++ )
    {
        q_int[j] = q_int[j]*delta_t3;
    }
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
