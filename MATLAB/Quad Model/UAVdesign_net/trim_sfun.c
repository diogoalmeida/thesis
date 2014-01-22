/*
 *      http://www.UAVdesign.net/
 *      S-function which is configured to execute "function-call" subsystems
 *      Compile with: mex 'trim_sfun.c' 'matrixlib.c'
 */

#define S_FUNCTION_NAME  trim_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "matrixlib.h"

/* Parameters */
#define NumFInputs(S)                ssGetSFcnParam(S, 0)
#define NumFOutputs(S)               ssGetSFcnParam(S, 1)
#define RunOnce(S)                   ssGetSFcnParam(S, 2)
#define SampleTime(S)                ssGetSFcnParam(S, 3)


#define NUM_DWORK_VEC  10
#define VAR_J           0
#define VAR_Jinv_lu     1
#define VAR_Jinv_piv    2
#define VAR_Jinv_x      3
#define VAR_c0          4
#define VAR_a           5
#define VAR_a0          6
#define VAR_a_left      7
#define VAR_a_right     8
#define VAR_deltac      9


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
    
    for (el=0; el<2; el++ )
    {
        if (mxIsEmpty(  ssGetSFcnParam(S, el)) ||
          mxIsSparse(   ssGetSFcnParam(S, el)) ||
          mxIsComplex(  ssGetSFcnParam(S, el)) ||
          mxIsLogical(  ssGetSFcnParam(S, el)) ||
          !mxIsNumeric( ssGetSFcnParam(S, el)) )
        {
          ssSetErrorStatus(S,"Both sizes have to be positive scalars.");
          return;
        }

        pr   = mxGetPr(ssGetSFcnParam(S, el));
        nEls = mxGetNumberOfElements(ssGetSFcnParam(S, el));

        if ((nEls != 1) || ((pr[0] <= 0)))
        {
          ssSetErrorStatus(S,"Both sizes have to be positive scalars.");
          return;
        }
    }
    
    if ((int)mxGetPr(NumFInputs(S))[0] != (int)mxGetPr(NumFOutputs(S))[0])
    {
        ssSetErrorStatus(S,"Both sizes have to be equal (until another search algorithm is implemented).");
        return;
    }
    
    if (!mxIsDouble(RunOnce(S))) 
    {
    	ssSetErrorStatus(S,"3rd parameter to S-function must be real \"RunOnce\"");
        return;
    }
    
    if (!mxIsDouble(SampleTime(S))) 
    {
    	ssSetErrorStatus(S,"4th parameter to S-function must be real \"sample time\"");
        return;
    }
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    int_T             Nc;
    int_T             Na;
    int_T             i;
    
    ssSetNumSFcnParams(S, 4);  /* Number of expected parameters */
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

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);
    
    Nc = (int)mxGetScalar(NumFInputs(S));
    Na = (int)mxGetScalar(NumFOutputs(S));
    
    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, Nc);
    ssSetInputPortWidth(S, 1, Na);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    
    if (!ssSetNumOutputPorts(S, 3)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1, Nc);
    ssSetOutputPortWidth(S, 2, Nc);

    ssSetNumSampleTimes(   S, 1);
    ssSetNumRWork(         S, 0);
    ssSetNumIWork(         S, 0);
    ssSetNumPWork(         S, 0);
    ssSetNumModes(         S, 0);
    ssSetNumNonsampledZCs( S, 0);
    
    /* variables */
    ssSetNumDWork(         S, NUM_DWORK_VEC);
    for (i=0; i<NUM_DWORK_VEC; i++)
    {
        switch (i)
        {
            case VAR_Jinv_piv:
                ssSetDWorkDataType(    S, i, SS_INT32);
                break;
            default:
                ssSetDWorkDataType(    S, i, SS_DOUBLE);
                break;
        }
        switch (i)
        {
            case VAR_c0:
                ssSetDWorkUsageType(S, i, SS_DWORK_USED_AS_DWORK);
                break;
            default:
                /* ssSetDWorkUsageType(S, i, SS_DWORK_USED_AS_SCRATCH); */
                ssSetDWorkUsageType(S, i, SS_DWORK_USED_AS_DWORK);
                break;
        }
        
    }

    ssSetDWorkWidth(       S, VAR_J          , Na*Nc);
    ssSetDWorkWidth(       S, VAR_Jinv_lu    , Nc*Nc);
    ssSetDWorkWidth(       S, VAR_Jinv_piv   , Nc   );
    ssSetDWorkWidth(       S, VAR_Jinv_x     , Nc*Nc);
    ssSetDWorkWidth(       S, VAR_c0         , Nc   );
    ssSetDWorkWidth(       S, VAR_a          , Na   );
    ssSetDWorkWidth(       S, VAR_a0         , Na   );
    ssSetDWorkWidth(       S, VAR_a_left     , Na   );
    ssSetDWorkWidth(       S, VAR_a_right    , Na   );
    ssSetDWorkWidth(       S, VAR_deltac     , Nc   );

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    
    ssSetCallSystemOutput(S,0);  /* call on first element */
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    Initializes the states
 */

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x = ssGetRealDiscStates(S);
    x[0] = 0;
}



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    Issue ssCallSystemWithTid on 1st or 2nd output element of 1st output port
 *    and then update 2nd output port with the state.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T            *x     = ssGetRealDiscStates(S);
    InputRealPtrsType c0Ptrs = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType aPtrs  = ssGetInputPortRealSignalPtrs(S,1);
    real_T            *c_final   = ssGetOutputPortRealSignal(S,1);
    real_T            *c     = ssGetOutputPortRealSignal(S,2);
    int_T             Nc     = ssGetInputPortWidth(S, 0);
    int_T             Na     = ssGetInputPortWidth(S, 1);
    int_T             i, j, k;
    int_T             n;
    real_T            c_disturb;
    real_T            *src;
    real_T            *src2;
    real_T            *dst;
    int_T             dims[3];

    /*
     * ssCallSystemWithTid is used to execute a function-call subsystem. The
     * 2nd argument is the element of the 1st output port index which
     * connected to the function-call subsystem. Function-call subsystems
     * can be driven by the first output port of s-function blocks.
     */
    
    if (x[0] == 0)
    {
        /* c0 = c0_input initially */
        dst = (real_T*) ssGetDWork(S, VAR_c0);
        for (n=0; n<Nc; n++)
        {
            *dst++ = *c0Ptrs[n];
        }
        
        for( i=0; i<40; i++)
        {
            /* find the corresponding function output a0=f(c0)*/
            src = (real_T*) ssGetDWork(S, VAR_c0);
            for (n=0; n<Nc; n++)
            {
                c[n] = src[n];
            }
            if (!ssCallSystemWithTid(S,0,tid)) {
                /* Error occurred which will be reported by Simulink */
                return;
            }
            dst = (real_T*) ssGetDWork(S, VAR_a0);
            for (n=0; n<Na; n++)
            {
                *dst++ = *aPtrs[n];
            }
            
            /* calculate J */
            /* Iterate through each column of J */
            for (k=0; k<Nc; k++)
            {
                /* disturbance amplitude at the inputs */
                c_disturb = 1e-5;
                
                /* left */
                src = (real_T*) ssGetDWork(S, VAR_c0);
                for (n=0; n<Nc; n++)
                {
                    c[n] = *src++;
                    if (n==k) c[n] -= c_disturb;
                }
                if (!ssCallSystemWithTid(S,0,tid)) {
                    /* Error occurred which will be reported by Simulink */
                    return;
                }
                dst = (real_T*) ssGetDWork(S, VAR_a_left);
                for (n=0; n<Na; n++)
                {
                    *dst++ = *aPtrs[n];
                }
                
                /* right */
                src = (real_T*) ssGetDWork(S, VAR_c0);
                for (n=0; n<Nc; n++)
                {
                    c[n] = *src++;
                    if (n==k) c[n] += c_disturb;
                }
                if (!ssCallSystemWithTid(S,0,tid)) {
                    /* Error occurred which will be reported by Simulink */
                    return;
                }
                dst = (real_T*) ssGetDWork(S, VAR_a_right);
                for (n=0; n<Na; n++)
                {
                    *dst++ = *aPtrs[n];
                }
                
                /* J[a, c] = (a_right - a_left)/(2 * c_disturb) */
                src = (real_T*) ssGetDWork(S, VAR_a_left);
                src2= (real_T*) ssGetDWork(S, VAR_a_right);
                dst = (real_T*) ssGetDWork(S, VAR_J);
                dst += k * Na;
                for (n=0; n<Na; n++)
                {
                    *dst = (*src2++ - *src++) / (2 * c_disturb);
                    dst++;
                }
            }

            /* deltac = inv(J) * a0 */
            dims[0] = Nc; dims[1] = Nc; dims[2] = 1;
            MatDivRR_Dbl((real_T*) ssGetDWork(S, VAR_deltac),
                    (real_T*) ssGetDWork(S, VAR_J),
                    (real_T*) ssGetDWork(S, VAR_a0),
                    (real_T*) ssGetDWork(S, VAR_Jinv_lu),
                    (int_T*)  ssGetDWork(S, VAR_Jinv_piv),
                    (real_T*) ssGetDWork(S, VAR_Jinv_x),
                    dims);
            
            /* deltac = -deltac */
            dst = (real_T*) ssGetDWork(S, VAR_deltac);
            for (j=0; j<Nc; j++)
            {
                *dst = -*dst;
                dst++;
            }
            
            /* c0 = c0 + delta_c */
            dst = (real_T*) ssGetDWork(S, VAR_c0);
            src = (real_T*) ssGetDWork(S, VAR_deltac);
            for (n=0; n<Nc; n++)
            {
                *dst++ += *src++;
            }
            
            /* stop condition */
            c_disturb = 0;
            src = (real_T*) ssGetDWork(S, VAR_deltac);
            for (n=0; n<Nc; n++)
            {
                c_disturb += *src * *src;
                src++;
            }
            if (c_disturb < 1e-30 * Nc) break;
        }

        /* call the function with c_final to allow the states to be updated if they are read in the application */
        src = (real_T*) ssGetDWork(S, VAR_c0);
        for (n=0; n<Nc; n++)
        {
            c[n] = *src;
            c_final[n] = *src++;
        }
        if (!ssCallSystemWithTid(S,0,tid))
        {
            /* Error occurred which will be reported by Simulink */
            return;
        }
    }
    
    src = (real_T*) ssGetDWork(S, VAR_c0);
    for (n=0; n<Nc; n++)
    {
        c[n] = *src;
        c_final[n] = *src++;
    }
}


/* Function: mdlUpdate ========================================================
 * Abstract:
 *    Increment the state for next time around (i.e. a counter).
 */
#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
    real_T *x = ssGetRealDiscStates(S);    

    UNUSED_ARG(tid); /* not used in single tasking mode */

    if (*mxGetPr(RunOnce(S)))
    {
        x[0] = 1;
    }
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S); /* unused input argument */
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
