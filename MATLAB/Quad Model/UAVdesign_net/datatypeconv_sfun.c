/*
 *      http://www.UAVdesign.net/
 *      S-function which converts between different data types 
 *      to provide the same binary representation. It ignores overflows.
 *      Compile with: mex 'datatypeconv_sfun.c'
 */

#define S_FUNCTION_NAME  datatypeconv_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

/* Parameters */
#define InputDimension(S)            ssGetSFcnParam(S, 0)
#define InputType(S)                 ssGetSFcnParam(S, 1)
#define OutputDimension(S)           ssGetSFcnParam(S, 2)
#define OutputType(S)                ssGetSFcnParam(S, 3)
#define SignExtend(S)                ssGetSFcnParam(S, 4)

#define intval(x) ((int)((x)+0.5))
#define min(X,Y) ((X) < (Y) ? (X) : (Y))
typedef enum
{
    sig_SS_DOUBLE = 0,
    sig_SS_SINGLE,
    sig_SS_INT8,
    sig_SS_UINT8,
    sig_SS_INT16,
    sig_SS_UINT16,
    sig_SS_INT32,
    sig_SS_UINT32,
    sig_SS_BOOLEAN,
    sig_SS_UDOUBLE,
    sig_SS_USINGLE
} sig_type;

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
    
    for (el=0; el<4; el++ )
    {
        if (mxIsEmpty(  ssGetSFcnParam(S, el)) ||
          mxIsSparse(   ssGetSFcnParam(S, el)) ||
          mxIsComplex(  ssGetSFcnParam(S, el)) ||
          mxIsLogical(  ssGetSFcnParam(S, el)) ||
          !mxIsNumeric( ssGetSFcnParam(S, el)) )
        {
          ssSetErrorStatus(S,"All parameters have to be positive scalars.");
          return;
        }

        pr   = mxGetPr(ssGetSFcnParam(S, el));
        nEls = mxGetNumberOfElements(ssGetSFcnParam(S, el));

        if ((nEls != 1) || ((intval(pr[0]) < 1)))
        {
          ssSetErrorStatus(S,"All parameters have to be positive scalars.");
          return;
        }
    }
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    int_T             Ni;
    int_T             No;
    sig_type          Ti;
    sig_type          To;
    int_T             param;
    
    ssSetNumSFcnParams(S, 5);  /* Number of expected parameters */
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

    for( param=0; param<5; param++ )
    {
        ssSetSFcnParamTunable(S,param,false);
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    
/* The popup menu provides the number below + 1
    SS_DOUBLE  =  0,    real_T
    SS_SINGLE  =  1,    real32_T
    SS_INT8    =  2,    int8_T
    SS_UINT8   =  3,    uint8_T
    SS_INT16   =  4,    int16_T
    SS_UINT16  =  5,    uint16_T
    SS_INT32   =  6,    int32_T
    SS_UINT32  =  7,    uint32_T
    SS_BOOLEAN =  8     boolean_T */
    
    Ni = intval(mxGetScalar(InputDimension(S)));
    No = intval(mxGetScalar(OutputDimension(S)));
    Ti = intval(mxGetScalar(InputType(S))) - 1;
    To = intval(mxGetScalar(OutputType(S))) - 1;
    if( To >= sig_SS_UDOUBLE ) To -= sig_SS_UDOUBLE;
    
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, Ni);
    ssSetInputPortDataType(S, 0, Ti);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, No);
    ssSetOutputPortDataType(S, 0, To);

    ssSetNumSampleTimes(   S, 1);
    ssSetNumRWork(         S, 0);
    ssSetNumIWork(         S, 0);
    ssSetNumPWork(         S, 0);
    ssSetNumDWork(         S, 0);
    ssSetNumModes(         S, 0);
    ssSetNumNonsampledZCs( S, 0);

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
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    const unsigned char *u   = ssGetInputPortSignal(S,0);
    unsigned char     *y     = ssGetOutputPortSignal(S,0);
    int_T             Ni     = intval(mxGetScalar(InputDimension(S)));
    int_T             No     = intval(mxGetScalar(OutputDimension(S)));
    int_T             Ti     = intval(mxGetScalar(InputType(S))) - 1;
    int_T             To     = intval(mxGetScalar(OutputType(S))) - 1;
    int_T             SE     = intval(mxGetScalar(SignExtend(S)));
    int_T             i;
    int_T             bit;
    int_T             byte;
    static const int  size[11] = {8,4,1,1,2,2,4,4,1,8,4};
    
    if( ((Ti == sig_SS_INT8) || (Ti == sig_SS_INT16) || (Ti == sig_SS_INT32)) && !(((unsigned char*)y)[Ni*size[Ti]-1] && 0x80) )
    {
        SE = 0;
    }
    
    if( (Ti != sig_SS_BOOLEAN ) && (To != sig_SS_BOOLEAN) )
    {
        memset(y, SE?0xFF:0, No*size[To]);
        memcpy(y, u, min(Ni*size[Ti], No*size[To]));
    }
    else if( (Ti == sig_SS_BOOLEAN ) && (To != sig_SS_BOOLEAN) )
    {
        memset(y, 0, No*size[To]);
        i = min(Ni, No*size[To]*8) - 1;
        byte = i / 8 + 1;
        bit = i % 8 + 1;
        while(byte--)
        {
            while(bit--)
            {
                y[byte] = (y[byte] << 1) | u[i--];
            }
            bit = 8;
        }
    }
    else if( (Ti != sig_SS_BOOLEAN ) && (To == sig_SS_BOOLEAN) )
    {
        memset(y, SE?1:0, No*size[To]);
        i = min(No, Ni*size[Ti]*8) - 1;
        byte = i / 8 + 1;
        bit = i % 8 + 1;
        while(byte--)
        {
            while(bit--)
            {
                y[i--] = ((u[byte] & (1 << bit)) != 0);
            }
            bit = 8;
        }
    }
    else
    {
        memset(y, 0, No*size[To]);
        memcpy(y, u, min(Ni*size[Ti], No*size[To]));
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
