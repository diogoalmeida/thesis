/*
 *      http://www.UAVdesign.net/
 *      S-function that outputs the simulation time
 *      Compile with: mex 'display_time_sfun.c'
 */

#define DEBUG_MODE
#define S_FUNCTION_NAME display_time_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

/* Parameters */
#define NPARAMS              1
#define SampleTime(S)        ssGetSFcnParam(S, 0)

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))


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
    const mxArray *pVal0 = ssGetSFcnParam(S,0);
    
    if ( (!IS_PARAM_DOUBLE(pVal0)) || (mxGetNumberOfElements(pVal0) != 1) )
    {
        ssSetErrorStatus(S, "Wrong input sample time.");
        return;
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
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
	if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S))
    {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
	} 
    else return; /* Parameter mismatch will be reported by Simulink */
#endif

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 0)) return;

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);

    ssSetNumSampleTimes(S, 1);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c. */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE));

} /* end mdlInitializeSizes */



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Two tasks: One continuous, one with discrete sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, *mxGetPr(SampleTime(S)));
    ssSetOffsetTime(S, 0, 0.0);

    ssSetModelReferenceSampleTimeDefaultInheritance(S);
} /* end mdlInitializeSampleTimes */



/* Function: mdlOutputs =======================================================
 * Abstract:
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T             *y    = ssGetOutputPortRealSignal(S,0);
    real_T             t; 
	
    t = ssGetTaskTime(S,0);
    y[0] = t;

#ifdef DEBUG_MODE
    if(ssIsMajorTimeStep(S))
    {
        ssPrintf("M  ");
    }
    else
    {
        ssPrintf("   ");
    }

    ssPrintf("   %g  ", t); 
    ssPrintf("\n");
#endif
} /* end mdlOutputs */



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
