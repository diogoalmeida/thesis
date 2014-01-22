/* *******************************************************************
   **** http://www.UAVdesign.net/                                 ****
   **** To build this mex function use:                           ****
   **** mex 'tomultimediafile.c'                                  ****
   ******************************************************************* */

#define S_FUNCTION_NAME tomultimediafile
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
#include <windows.h>
#include <stdio.h>
#else
#ifndef _LARGEFILE64_SOURCE
#define _LARGEFILE64_SOURCE
#endif
#endif

#define NPARAMS                 5
#define paramFileFormat         ssGetSFcnParam(S,0)
#define paramFileName           ssGetSFcnParam(S,1)
#define paramForceFrameRate     ssGetSFcnParam(S,2)
#define paramFrameRate          ssGetSFcnParam(S,3)
#define paramSampleTime         ssGetSFcnParam(S,4)

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define intval(x) ((int)((x)+0.5))

typedef struct
{
    uint32_T dwFourCC;
    uint32_T dwSize;
#ifdef AVI_SUPPORT
    unsigned char data[0];  /* contains headers or video/audio data */
#endif
} CHUNK;

typedef struct
{
    uint32_T dwList;
    uint32_T dwSize;
    uint32_T dwFourCC;
#ifdef AVI_SUPPORT
    unsigned char data[0];  /* contains Lists and Chunks */
#endif
} LIST;

typedef struct
{
#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    HANDLE        file;
#else
    int           file;
#endif
    LIST          avi_header;
    unsigned long dim_x;
    unsigned long dim_y;
    unsigned long dim_pixel;
    unsigned long frame_size;
} MMOBJECT, *pMMOBJECT;

static char msg[200];

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
#define sighandle(x) x

#else
#include <errno.h>
#include <fcntl.h>

#define sighandle(x) while( (x) == -1 && errno == EINTR) continue

#endif

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
    if ( !mxIsChar(paramFileName) || (mxGetNumberOfElements(paramFileName)) < 1 )
    {
        ssSetErrorStatus(S, "The File Name parameter must be a nonempty string.");
        return;
    }

    if ( (!IS_PARAM_DOUBLE(paramForceFrameRate)) || (mxGetNumberOfElements(paramForceFrameRate) != 1) )
    {
        ssSetErrorStatus(S, "Force Frame Rate has to be 0 or 1.");
        return;
    }
    
    if ( (!IS_PARAM_DOUBLE(paramFrameRate)) || (mxGetNumberOfElements(paramFrameRate) != 1) )
    {
        ssSetErrorStatus(S, "Wrong frame rate.");
        return;
    }

    if ( (!IS_PARAM_DOUBLE(paramSampleTime)) || (mxGetNumberOfElements(paramSampleTime) != 1) )
    {
        ssSetErrorStatus(S, "Wrong sample time.");
        return;
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
    ssSetNumDiscStates(S, 1);
    ssAllowSignalsWithMoreThan2D(S);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortDimensionInfo(S, 0, DYNAMIC_DIMENSION);
    ssSetInputPortDataType(S, 0, SS_UINT8 );
    ssSetInputPortComplexSignal(S, 0, 0);
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
    
    if (!ssSetNumOutputPorts(S, 0)) return;
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 1);
    
    /* Reserve place for C objects */
    ssSetNumPWork(S, 1);
    
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
/* Function: mdlSetInputPortDimensionInfo ====================================
 * Abstract:
 *    This routine is called with the candidate dimensions for an input port
 *    with unknown dimensions. If the proposed dimensions are acceptable, the
 *    routine should go ahead and set the actual port dimensions.
 *    If they are unacceptable an error should be generated via
 *    ssSetErrorStatus.
 *    Note that any other input or output ports whose dimensions are
 *    implicitly defined by virtue of knowing the dimensions of the given port
 *    can also have their dimensions set.
 */
static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if( (dimsInfo->numDims < 2) || (dimsInfo->numDims > 3) )
    {
        ssSetErrorStatus(S, "The number of dimensions for the input signal has to be 2 or 3.");
        return;
    }
    
    if( (dimsInfo->numDims == 3) && (dimsInfo->dims[2] != 3) )
    {
        ssSetErrorStatus(S, "The 3rd dimension has to have 3 components.");
        return;
    }
    
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
} /* end mdlSetInputPortDimensionInfo */

# define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
  /* Function: mdlSetDefaultPortDimensionInfo ===========================================
   * Abstract:
   *   In case no ports were specified
   */
  static void mdlSetDefaultPortDimensionInfo(SimStruct *S)
  {
      DECL_AND_INIT_DIMSINFO(di);
      int_T dims[3];

      di.numDims = 3;
      dims[0] = 480;
      dims[1] = 640;
      dims[2] = 3;
      di.dims = dims;
      di.width = dims[0]*dims[1]*dims[2];
      ssSetInputPortDimensionInfo(S, 0, &di);
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
    pMMOBJECT mmobj;
    char* file_name;
    int file_name_len;
    
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }
    
    /* Store new C object in the pointers vector */
    mmobj = (pMMOBJECT) calloc(1, sizeof(MMOBJECT));
    ssGetPWork(S)[0] = mmobj;
#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    mmobj->file = INVALID_HANDLE_VALUE;
#else
    mmobj->file = -1;
#endif

    file_name_len = mxGetM(paramFileName) * mxGetN(paramFileName) + 1;
    file_name = (char*) malloc(file_name_len);
    mxGetString(paramFileName, file_name, file_name_len);
    
    mmobj->dim_pixel = (ssGetInputPortNumDimensions(S, 0) == 2 ? 1 : 3);
    mmobj->dim_x = ssGetInputPortDimensionSize(S, 0, 1);
    mmobj->dim_y = ssGetInputPortDimensionSize(S, 0, 0);
    mmobj->frame_size = ssGetInputPortWidth(S, 0);

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    if( (mmobj->file = CreateFile(file_name, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL)) == INVALID_HANDLE_VALUE )
    {
        _snprintf(msg, 200, "Could not open the MM file %s for writing. Error = %d.", file_name, GetLastError());
        ssSetErrorStatus(S,msg);
        free(file_name);
        return;
    }    
#else
    if( (mmobj->file = open(file_name, O_WRONLY | O_CREAT | O_NOCTTY | O_TRUNC | O_LARGEFILE)) == -1 )
    {
        snprintf(msg, 200, "Could not open the MM file %s for writing. Error = %d.", file_name, errno);
        ssSetErrorStatus(S,msg);
        free(file_name);
        return;
    }
#endif
    free(file_name);
    
    /* avi_header */
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
}

#define MDL_UPDATE
#if defined(MDL_UPDATE)
/* Function: mdlUpdate ======================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Discrete states are typically updated here, but this function is useful
 *    for performing any tasks that should only take place once per
 *    integration step.
 */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    int err;
    uint8_T *pu;
    pMMOBJECT mmobj;
#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    DWORD written;
#endif
    UNUSED_ARG(tid); /* not used in single tasking mode */
    
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }
    
    /* Retrieve C object from the pointers vector */
    mmobj = (pMMOBJECT) ssGetPWork(S)[0];
    
    if( !mmobj )
    {
        ssSetErrorStatus(S,"The MM object has not been initialized.");
        return;
    }
    
    /* Write the data to the MM file */
    pu = (uint8_T *)ssGetInputPortSignal(S, 0);

    /* Send the data */
#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    if( !WriteFile(mmobj->file, pu, mmobj->frame_size, &written, NULL) )
#else
    sighandle(err = write(mmobj->file, pu, mmobj->frame_size));
    if( err == -1 )
#endif
    {
        sprintf(msg, "Could not write to the MM file. Error=%d", 
#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
                GetLastError()
#else
                errno
#endif
                );
        ssSetErrorStatus(S,msg);
        return;
    }
}
#endif /* MDL_UPDATE */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
#define MDL_TERMINATE
static void mdlTerminate(SimStruct *S)
{
    pMMOBJECT mmobj;
    
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }

    /* Retrieve and destroy C object */
    mmobj = (pMMOBJECT) ssGetPWork(S)[0];
    
    if( mmobj )
    {

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
        if( mmobj->file != INVALID_HANDLE_VALUE )
        {
            CloseHandle(mmobj->file);
        }
#else
        if( mmobj->file != -1 )
        {
            close(mmobj->file);
        }
#endif
        free(mmobj);
        ssGetPWork(S)[0] = 0;
    }
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
