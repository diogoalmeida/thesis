/*
 *      http://www.UAVdesign.net/
 *      S-function which is configured to execute "function-call" subsystems
 *      Compile with: mex 'jacobian_sfun.c'
 */

/* #define DEBUG_MODE */
#define S_FUNCTION_NAME  jacobian_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

/* Parameters */
#define NumFInputs(S)                ssGetSFcnParam(S, 0)
#define NumFOutputs(S)               ssGetSFcnParam(S, 1)
#define RunOnce(S)                   ssGetSFcnParam(S, 2)
#define SampleTime(S)                ssGetSFcnParam(S, 3)


#define NUM_DWORK_VEC   4
#define VAR_J           0
#define VAR_c0          1
#define VAR_a_left      2
#define VAR_a_right     3


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

    for( i=0; i<4; i++ )
    {
        ssSetSFcnParamTunable(S,i,false);
    }

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
    ssSetOutputPortMatrixDimensions(S, 1, Na, Nc);
    ssSetOutputPortWidth(S, 2, Nc);

    ssSetNumSampleTimes(   S, 1);
    ssSetNumRWork(         S, 0);
    ssSetNumIWork(         S, 0);
    ssSetNumPWork(         S, 0);
    ssSetNumModes(         S, 0);
    ssSetNumNonsampledZCs( S, 0);
    
    /* variables */
    ssSetNumDWork(         S, NUM_DWORK_VEC);
    ssSetDWorkWidth(       S, VAR_J          , Na*Nc);
    ssSetDWorkWidth(       S, VAR_c0         , Nc   );
    ssSetDWorkWidth(       S, VAR_a_left     , Na   );
    ssSetDWorkWidth(       S, VAR_a_right    , Na   );

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
    real_T            *J     = ssGetOutputPortSignal(S,1);
    real_T            *c     = ssGetOutputPortRealSignal(S,2);
    int_T             Nc     = ssGetInputPortWidth(S, 0);
    int_T             Na     = ssGetInputPortWidth(S, 1);
    int_T             n, k;
    real_T            c_disturb;
    real_T            *src;
    real_T            *src2;
    real_T            *dst;

    /*
     * ssCallSystemWithTid is used to execute a function-call subsystem. The
     * 2nd argument is the element of the 1st output port index which
     * connected to the function-call subsystem. Function-call subsystems
     * can be driven by the first output port of s-function blocks.
     */
    
    if ((x[0] == 0) && ssIsMajorTimeStep(S))
    {
        /* c0 = c0_input initially */
        dst = (real_T*) ssGetDWork(S, VAR_c0);
        for (n=0; n<Nc; n++)
        {
            *dst++ = *c0Ptrs[n];
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
                /* return; */
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
                /* return; */
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
#ifdef DEBUG_MODE
                ssPrintf("  %f", *dst);
#endif
                dst++;
            }
#ifdef DEBUG_MODE
            ssPrintf("\n");
#endif
        }

        /* call the function with c_final to allow the states to be updated if they are read in the application */
        src = (real_T*) ssGetDWork(S, VAR_c0);
        for (n=0; n<Nc; n++)
        {
            c[n] = *src++;
        }
        if (!ssCallSystemWithTid(S,0,tid))
        {
            /* Error occurred which will be reported by Simulink */
            /* return; */
        }
    }
    
    /* Output the Jacobian */
    src = (real_T*) ssGetDWork(S, VAR_J);
    dst = J;
    for (k=0; k<Nc*Na; k++)
    {
        *J++ = *src++;
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
