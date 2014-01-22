/*
 *      http://www.UAVdesign.net/
 *      S-function that implements a continuous-discrete Extended Kalman Filter
 *      Compile with: mex 'kalman_filter_sfun.c'
 */

/* #define DEBUG_MODE */
#define S_FUNCTION_NAME kalman_filter_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

/* Parameters */
#define NPARAMS              1
#define SampleTime(S)        ssGetSFcnParam(S, 0)

/* Input ports */
#define NINPORTS             3
#define INPORT_x0            0
#define INPORT_xkp           1
#define INPORT_xdot          2

/* Output ports */
#define NOUTPORTS            3
#define OUTPORT_fncall       0
#define OUTPORT_xkm          1
#define OUTPORT_x            2

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define intval(x) ((int)((x)+0.5))


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
    if ( (!IS_PARAM_DOUBLE(SampleTime(S))) || (mxGetNumberOfElements(SampleTime(S)) != 1) )
    {
        ssSetErrorStatus(S, "Wrong discrete sample time.");
        return;
    }
    
    if ( mxGetPr(SampleTime(S))[0] <= 0 )
    {
        ssSetErrorStatus(S, "The discrete sample time must be a positive number.");
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
    real_T       sample_time;
    
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S))
    {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
	} 
    else return; /* Parameter mismatch will be reported by Simulink */
#endif

    ssSetSFcnParamTunable(S,0,false);

    sample_time = mxGetScalar(SampleTime(S));
    ssSetNumContStates(S, DYNAMICALLY_SIZED);
    ssSetNumDiscStates(S, 0);
    ssSetNumIWork(S, 1);
    

    if (!ssSetNumInputPorts(S, NINPORTS)) return;
    ssSetInputPortWidth(             S, INPORT_x0, DYNAMICALLY_SIZED);
    ssSetInputPortDirectFeedThrough( S, INPORT_x0, 1);
/*    ssSetInputPortSampleTime(        S, INPORT_x0, CONTINUOUS_SAMPLE_TIME); */
/*    ssSetInputPortOffsetTime(        S, INPORT_x0, 0.0); */
    
    ssSetInputPortWidth(             S, INPORT_xkp, DYNAMICALLY_SIZED);
    ssSetInputPortRequiredContiguous(S, INPORT_xkp, 1);
    ssSetInputPortDirectFeedThrough( S, INPORT_xkp, 1);
/*    ssSetInputPortSampleTime(        S, INPORT_xkp, sample_time); */
/*    ssSetInputPortOffsetTime(        S, INPORT_xkp, 0.0); */

    ssSetInputPortWidth(             S, INPORT_xdot, DYNAMICALLY_SIZED);
    ssSetInputPortRequiredContiguous(S, INPORT_xdot, 1);
    ssSetInputPortDirectFeedThrough( S, INPORT_xdot, 0);
/*    ssSetInputPortSampleTime(        S, INPORT_xdot, CONTINUOUS_SAMPLE_TIME); */
/*    ssSetInputPortOffsetTime(        S, INPORT_xdot, 0.0); */

    if (!ssSetNumOutputPorts(S, NOUTPORTS)) return;
    ssSetOutputPortWidth(            S, OUTPORT_fncall, 1);
    
    ssSetOutputPortWidth(            S, OUTPORT_xkm, DYNAMICALLY_SIZED);
/*    ssSetOutputPortSampleTime(       S, OUTPORT_xkm, sample_time); */
/*    ssSetOutputPortOffsetTime(       S, OUTPORT_xkm, 0.0); */
    
    ssSetOutputPortWidth(            S, OUTPORT_x, DYNAMICALLY_SIZED);
/*    ssSetOutputPortSampleTime(       S, OUTPORT_x, CONTINUOUS_SAMPLE_TIME); */
/*    ssSetOutputPortOffsetTime(       S, OUTPORT_x, 0.0); */
    
    ssSetNumSampleTimes(S, 2);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c. */
/*    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                       SS_OPTION_PORT_SAMPLE_TIMES_ASSIGNED)); */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE));

} /* end mdlInitializeSizes */

#if defined(MATLAB_MEX_FILE)
# define MDL_SET_INPUT_PORT_WIDTH
  static void mdlSetInputPortWidth(SimStruct *S, int_T port, int_T inputPortWidth)
  {
      if( ((ssGetInputPortWidth(S, INPORT_x0  ) != DYNAMICALLY_SIZED) && (ssGetInputPortWidth(S, INPORT_x0  ) != inputPortWidth)) || \
          ((ssGetInputPortWidth(S, INPORT_xkp ) != DYNAMICALLY_SIZED) && (ssGetInputPortWidth(S, INPORT_xkp ) != inputPortWidth)) || \
          ((ssGetInputPortWidth(S, INPORT_xdot) != DYNAMICALLY_SIZED) && (ssGetInputPortWidth(S, INPORT_xdot) != inputPortWidth)) )
      {
          ssSetErrorStatus(S,"x0, xkp and xdot must have the same size");
          return;
      }
      ssSetInputPortWidth(S, INPORT_x0, inputPortWidth);
      ssSetInputPortWidth(S, INPORT_xkp, inputPortWidth);
      ssSetInputPortWidth(S, INPORT_xdot, inputPortWidth);
      ssSetOutputPortWidth(S, OUTPORT_xkm, inputPortWidth);
      ssSetOutputPortWidth(S, OUTPORT_x, inputPortWidth);
  }

# define MDL_SET_OUTPUT_PORT_WIDTH
  static void mdlSetOutputPortWidth(SimStruct *S, int_T port, int_T outputPortWidth)
  {
  }

# define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
  /* Function: mdlSetDefaultPortDimensionInfo ===========================================
   * Abstract:
   *   In case no ports were specified
   */
  static void mdlSetDefaultPortDimensionInfo(SimStruct *S)
  {
      ssSetInputPortWidth(S, INPORT_x0, 2);
      ssSetInputPortWidth(S, INPORT_xkp, 2);
      ssSetInputPortWidth(S, INPORT_xdot, 2);
      ssSetOutputPortWidth(S, OUTPORT_xkm, 2);
      ssSetOutputPortWidth(S, OUTPORT_x, 2);
  }

# define MDL_SET_WORK_WIDTHS
  static void mdlSetWorkWidths(SimStruct *S)
  {
      ssSetNumContStates(S, ssGetInputPortWidth(S,INPORT_x0));
  }
#endif



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Two tasks: One continuous, one with discrete sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

    ssSetSampleTime(S, 1, mxGetScalar(SampleTime(S)));
    ssSetOffsetTime(S, 1, 0.0);
    
    ssSetCallSystemOutput(S,0);  /* call on first element */
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
} /* end mdlInitializeSampleTimes */


#define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ==========================================
 * Abstract:
 *      Initalize the states, numerator and denominator coefficients.
 */
static void mdlInitializeConditions(SimStruct *S)
{
    int_T              *rst_xC = ssGetIWork(S);
    
    *rst_xC = 1;
} /* end mdlInitializeConditions */



/* Function: mdlOutputs =======================================================
 * Abstract:
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    int_T              *rst_xC = ssGetIWork(S);
    real_T             *xC     = ssGetContStates(S);
    InputRealPtrsType  x0Ptrs  = ssGetInputPortRealSignalPtrs(S, INPORT_x0);
    int_T              Nstates = ssGetInputPortWidth(S, INPORT_x0);
    int_T              i;
    real_T             t; 
	
    if( *rst_xC )
    {
        *rst_xC = 0;
        for(i=0; i<Nstates; i++)
        {
            xC[i] = *x0Ptrs[i];
        }
        
        ssSetSolverNeedsReset(S);
    }
    
    if( ssIsSampleHit(S,1,tid) )
    {
        const real_T       *xkp    = ssGetInputPortRealSignal(S, INPORT_xkp);
        real_T             *xkm    = ssGetOutputPortRealSignal(S, OUTPORT_xkm);
    
        for(i=0; i<Nstates; i++)
        {
            xkm[i] = xC[i];
        }
        
        if (!ssCallSystemWithTid(S, 0, tid))
        {
            return; /* Error occurred which will be reported by Simulink */
        }
        
        for(i=0; i<Nstates; i++)
        {
            xC[i] = xkp[i];
        }
        
        ssSetSolverNeedsReset(S);
    }
    
    if(ssIsContinuousTask(S,tid))
    {
        real_T             *x      = ssGetOutputPortRealSignal(S, OUTPORT_x);
        for(i=0; i<Nstates; i++)
        {
            x[i] = xC[i];
        }
    }

#ifdef DEBUG_MODE
    if(ssIsMajorTimeStep(S))
    {
        ssPrintf("M  ");
    }
    else
    {
        ssPrintf("   ");
    }
    if(ssIsSpecialSampleHit(S,1,0,tid)) {
    	t = ssGetTaskTime(S,0);
		ssPrintf("CD  %g  ", t); 
    }
    if(ssIsContinuousTask(S,tid)) { 
		t = ssGetTaskTime(S,0); 
		ssPrintf("C   %g  ", t); 
	} 
	if(ssIsSampleHit(S,1,tid)) { 
		t = ssGetTaskTime(S,1); 
		ssPrintf("D   %g  ",t); 
	}
    ssPrintf("\n");
#endif
} /* end mdlOutputs */


#define MDL_DERIVATIVES
/* Function: mdlDerivatives =================================================
 * Abstract:
 *      xdot = U
 */
static void mdlDerivatives(SimStruct *S)
{
    real_T             *dx     = ssGetdX(S);
    const real_T       *xdot   = ssGetInputPortRealSignal(S, INPORT_xdot);
    int_T              Nstates = ssGetInputPortWidth(S, INPORT_x0);
    int_T              i;

    /* xdot=U */
    for(i=0; i<Nstates; i++)
    {
        dx[i] = xdot[i];
    }
} /* end mdlDerivatives */


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
