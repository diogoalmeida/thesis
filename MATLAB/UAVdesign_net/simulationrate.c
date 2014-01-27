/* ********************************************************************************
   **** http://www.UAVdesign.net/                                              ****
   **** To build this mex function use:                                        ****
   **** in Windows:                                                            ****
   ****     mex 'simulationrate.c'                                             ****
   **** in Linux:                                                              ****
   ****     mex 'simulationrate.c' -lrt                                        ****
   ****     and add/modify LIBS = -lrt to the library section in ???_unix.tmf  ****
   ******************************************************************************** */


#define S_FUNCTION_NAME simulationrate
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif  /* _WIN32 */

#define NPARAMS                 1
#define paramSimRate            ssGetSFcnParam(S,0)

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define intval(x) ((int)((x)+((x) >= 0 ? 0.5 : -0.5)))

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

    if ( (!IS_PARAM_DOUBLE(paramSimRate)) || (mxGetNumberOfElements(paramSimRate) != 1) )
    {
        ssSetErrorStatus(S, "The simulation sample period has to be a scalar.");
        return;
    }
    
    if( mxGetPr(paramSimRate)[0] <= 0 )
    {
        ssSetErrorStatus(S, "The simulation sample period has to be positive.");
        return;
    }
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{
    int_T param;
    
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

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 0)) return;
    if (!ssSetNumOutputPorts(S, 0)) return;
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE );
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
	ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
	ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

/* Function: mdlStart =======================================================
 * Abstract:
 * This function is called once at start of model execution. If you
 * have states that should be initialized once, this is the place
 * to do it.
 */
#define MDL_START
static void mdlStart(SimStruct *S)
{

}


/* Function: mdlOutputs =======================================================
 *
 */
#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
{
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }
    #ifdef _WIN32    
    Sleep(mxGetPr(paramSimRate)[0]*1000.0);
    #else
	usleep(mxGetPr(paramSimRate)[0]*1000000.0);
    #endif
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

}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
