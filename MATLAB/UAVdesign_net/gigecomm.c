/* *************************************************************************************
   **** http://www.UAVdesign.net/                                                   ****
   **** This S-function needs the PvApi.h and the correct libPvAPI.a, libPvAPI.so   ****
   **** or PvAPI.dll files from the Allied Vision Technologies PvAPI SDK.           ****
   **** These files are already included in the UAVdesign_net library folder.       ****
   **** Newer versions of the SDK can be downloaded from                            ****
   ****     http://www.alliedvisiontec.com/us/products/software/avt-pvapi-sdk.html  ****
   **** If the PvApi.h file is replaced with a newer version, please run in Matlab: ****
   ****     cd_c_comments('PvApi.h', 'PvApi.h')                                     ****
   **** To build this mex function use:                                             ****
   **** For loading of the library at runtime:                                      ****
   ****     make sure that LOAD_PVLIB_AT_RUNTIME is defined in the code below       ****
   ****     mex 'gigecomm.c' -lrt -ldl                                              ****
   **** For dynamic linking:                                                        ****
   ****     comment out the LOAD_PVLIB_AT_RUNTIME definition                        ****
   ****     mex 'gigecomm.c' -L(path to the UAVdesign_net folder) -lPvAPI           ****
   **** For static linking:                                                         ****
   ****     comment out the LOAD_PVLIB_AT_RUNTIME definition                        ****
   ****     mex 'gigecomm.c' (path to the UAVdesign_net folder)/libPvAPI.a -lrt     ****
   ************************************************************************************* */

#define S_FUNCTION_NAME gigecomm
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

/* The following #define enables the loading pf the PvLib library at runtime
 * instead of it being linked.
 * Comment it out in order to link the library statically or dynamically
 */
#define LOAD_PVLIB_AT_RUNTIME

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(__LCC__)
#include <windows.h>
#include <winsock.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#else
#ifndef __USE_POSIX199309
#define __USE_POSIX199309
#endif
#include <unistd.h>
#include <signal.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <sys/times.h>
#include <dlfcn.h>
#endif

#include "gigecomm.h"

#define NPARAMS                 7
#define paramHostName           ssGetSFcnParam(S,0)
#define paramImageSizeXYN       ssGetSFcnParam(S,1)
#define paramPixelFormat        ssGetSFcnParam(S,2)
#define paramDumpRawData        ssGetSFcnParam(S,3)
#define paramSoftwareTrigger    ssGetSFcnParam(S,4)
#define paramTimeout            ssGetSFcnParam(S,5)
#define paramSampleTime         ssGetSFcnParam(S,6)


#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define intval(x) ((int)((x)+0.5))

/* Local functions */
static void PVDECL callback_fn(tPvFrame* Frame);
#ifdef LOAD_PVLIB_AT_RUNTIME
static lib_handle load_library(char* errmsg);
static void close_library(lib_handle PvLib);
#endif

/* Static variables common to all instances */
static int num_instances = 0;
static char msg[200];

#ifdef LOAD_PVLIB_AT_RUNTIME
/* Runtime loading of the PvLib library */
static lib_handle               PvLib;
#if defined(__LCC__)
static PVDECL tPvInitializeNoDiscovery ss_PvInitializeNoDiscovery;
static PVDECL tPvCameraOpenByAddr      ss_PvCameraOpenByAddr;
static PVDECL tPvCaptureStart          ss_PvCaptureStart;
static PVDECL tPvAttrUint32Set         ss_PvAttrUint32Set;
static PVDECL tPvAttrEnumSet           ss_PvAttrEnumSet;
static PVDECL tPvCommandRun            ss_PvCommandRun;
static PVDECL tPvAttrUint32Get         ss_PvAttrUint32Get;
static PVDECL tPvCaptureQueueFrame     ss_PvCaptureQueueFrame;
static PVDECL tPvCaptureEnd            ss_PvCaptureEnd;
static PVDECL tPvCaptureQueueClear     ss_PvCaptureQueueClear;
static PVDECL tPvCameraClose           ss_PvCameraClose;
static PVDECL tPvUnInitialize          ss_PvUnInitialize;
#else
static tPvInitializeNoDiscovery ss_PvInitializeNoDiscovery;
static tPvCameraOpenByAddr      ss_PvCameraOpenByAddr;
static tPvCaptureStart          ss_PvCaptureStart;
static tPvAttrUint32Set         ss_PvAttrUint32Set;
static tPvAttrEnumSet           ss_PvAttrEnumSet;
static tPvCommandRun            ss_PvCommandRun;
static tPvAttrUint32Get         ss_PvAttrUint32Get;
static tPvCaptureQueueFrame     ss_PvCaptureQueueFrame;
static tPvCaptureEnd            ss_PvCaptureEnd;
static tPvCaptureQueueClear     ss_PvCaptureQueueClear;
static tPvCameraClose           ss_PvCameraClose;
static tPvUnInitialize          ss_PvUnInitialize;
#endif
#else
#define ss_PvInitializeNoDiscovery PvInitializeNoDiscovery
#define ss_PvCameraOpenByAddr      PvCameraOpenByAddr
#define ss_PvCaptureStart          PvCaptureStart
#define ss_PvAttrUint32Set         PvAttrUint32Set
#define ss_PvAttrEnumSet           PvAttrEnumSet
#define ss_PvCommandRun            PvCommandRun
#define ss_PvAttrUint32Get         PvAttrUint32Get
#define ss_PvCaptureQueueFrame     PvCaptureQueueFrame
#define ss_PvCaptureEnd            PvCaptureEnd
#define ss_PvCaptureQueueClear     PvCaptureQueueClear
#define ss_PvCameraClose           PvCameraClose
#define ss_PvUnInitialize          PvUnInitialize
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
    unsigned int i;
    if ( !mxIsChar(paramHostName) || (mxGetNumberOfElements(paramHostName) < 1) || (mxGetNumberOfElements(paramHostName) > 100) )
    {
        ssSetErrorStatus(S, "The Host Name parameter must be a nonempty string with max. 100 elements.");
        return;
    }

    if ( (!IS_PARAM_DOUBLE(paramImageSizeXYN)) || (mxGetNumberOfElements(paramImageSizeXYN) < 2)  || (mxGetNumberOfElements(paramImageSizeXYN) > 3) )
    {
        ssSetErrorStatus(S, "The image size must have two elements [X Y] or three elements [X Y frames].");
        return;
    }
    else
    {
        for( i=0; i<mxGetNumberOfElements(paramImageSizeXYN); i++ )
        {
            if( intval(mxGetPr(paramImageSizeXYN)[i]) <= 0 )
            {
                ssSetErrorStatus(S, "The image dimensions have to be positive integers.");
                return;
            }
        }
    }
    
    if ( (!IS_PARAM_DOUBLE(paramPixelFormat)) || (mxGetNumberOfElements(paramPixelFormat) != 1) )
    {
        ssSetErrorStatus(S, "Pixel format must be a scalar");
        return;
    } 
    else if (intval(mxGetPr(paramPixelFormat)[0]) != 1 && intval(mxGetPr(paramPixelFormat)[0]) != 5)
    {
        ssSetErrorStatus(S, "Pixel format must be Mono8(1) or Rgb24(5)");
        return;
    }
    
    if ( (!IS_PARAM_DOUBLE(paramTimeout)) || (mxGetNumberOfElements(paramTimeout) != 1) )
    {
        ssSetErrorStatus(S, "Timeout must be a scalar");
        return;
    } 
    else if (mxGetPr(paramTimeout)[0] < 0)
    {
        ssSetErrorStatus(S, "Timeout must be non-negative");
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
#   if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S))
    {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
	} 
    else return; /* Parameter mismatch will be reported by Simulink */
#   endif

    for( param=0; param<NPARAMS; param++ )
    {
        ssSetSFcnParamTunable(S,param,false);
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);
    ssAllowSignalsWithMoreThan2D(S);
    if (!ssSetNumInputPorts(S, 0)) return;
    if (!ssSetNumOutputPorts(S, 2)) return;
    
    {
        DECL_AND_INIT_DIMSINFO(di);
        int_T dims[3];

        di.numDims = (intval(mxGetPr(paramPixelFormat)[0]) == 1 ? 2 : 3);
        dims[0] = intval(mxGetPr(paramImageSizeXYN)[1]);
        dims[1] = intval(mxGetPr(paramImageSizeXYN)[0]);
        dims[2] = (intval(mxGetPr(paramPixelFormat)[0]) == 1 ? 1 : 3);
        if ((mxGetNumberOfElements(paramImageSizeXYN) == 3) && (intval(mxGetPr(paramImageSizeXYN)[2])) > 1)
        {
            dims[1] *= intval(mxGetPr(paramImageSizeXYN)[2]);
        }
        di.dims = dims;
        di.width = dims[0]*dims[1]*dims[2];
        ssSetOutputPortDimensionInfo(S, 0, &di);
    }
    ssSetOutputPortDataType( S, 0, SS_UINT8 );
    ssSetOutputPortComplexSignal(S, 0, 0);
    ssSetOutputPortOptimOpts(S, 0, SS_NOT_REUSABLE_AND_GLOBAL);
    
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortDataType( S, 1, SS_UINT32 );
    ssSetOutputPortComplexSignal(S, 1, 0);
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    
    /* Reserve place for C objects */
    ssSetNumPWork(S, 1);
    
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}


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
    tCamera* Camera;
    char host_name[101];
    int host_len;
    unsigned long host;
    struct hostent *hent;
    unsigned int i;
    unsigned long FrameSize;
    const int pixel_size[9] = {1, 2, 1, 2, 3, 6, 0, 0, 0};
    const char* const pixel_format[9] = {"Mono8", "Mono16", "Bayer8", "Bayer16", "Rgb24", "Rgb48", "Yuv411", "Yuv422", "Yuv444"};
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    WSADATA wsa_data;
#   endif

    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }

    num_instances++;

    if( num_instances == 1 )
    {
#       if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
        /* Activate the Winsock DLL */
        if (WSAStartup(MAKEWORD(2,2),&wsa_data) != 0)
        {
            ssSetErrorStatus(S, "Can not initialize the Winsock DLL.");
            return;
        }
#       endif
        
#       ifdef LOAD_PVLIB_AT_RUNTIME
        /* Load the shared library */
        if( !(PvLib = load_library(msg)) )
        {
            ssSetErrorStatus(S, msg);
            return;
        }
#       endif

        /* Initialize the camera */
        if( ss_PvInitializeNoDiscovery() != ePvErrSuccess )
        {
            ssSetErrorStatus(S,"Could not initialize the PvAPI library. \r\nCheck that the shared library is in the same folder with the model.");
            return;
        }
    }
    
    /* Try to find the IP address for the hostname */
    host_len = mxGetM(paramHostName) * mxGetN(paramHostName) + 1;
    if( host_len  > sizeof(host_name) ) host_len = sizeof(host_name);
    mxGetString(paramHostName, host_name, host_len);
    
    hent = gethostbyname(host_name);
    if (hent != NULL)
    {
        memcpy((char *)&host, (char *)hent->h_addr_list[0], hent->h_length);
    }
    else
    {
        /* If hostname does not exist try as IP address */
        host = inet_addr(host_name);
        if (host==INADDR_NONE)
        {
            ssSetErrorStatus(S,"Invalid camera IP or the host name does not exist.");
            return;
        }
    }

    /* Store new C object in the pointers vector */
    Camera = (tCamera*) calloc(1,sizeof(tCamera));
    ssGetPWork(S)[0] = Camera;
    
    /* Initialize the thread synchronization objects */
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    InitializeCriticalSection(&(Camera->csec));
    Camera->csec_init = 1;
    if( (Camera->event = CreateEvent( NULL, FALSE, FALSE, NULL )) == NULL)
    {
        ssSetErrorStatus(S, "Can not initialize the event variable.");
        return;
    }
#   else
    if( pthread_mutex_init(&(Camera->mutex),NULL) )
    {
        ssSetErrorStatus(S, "Can not initialize the mutex.");
        return;
    }
    Camera->mutex_init = 1;

    if( pthread_cond_init( &(Camera->cond),NULL) )
    {
        ssSetErrorStatus(S, "Can not initialize the condition variable.");
        return;
    }
    Camera->cond_init = 1;
#   endif

    /* Populate the Camera structure fields */
    Camera->dim_x = intval(mxGetPr(paramImageSizeXYN)[0]);
    Camera->dim_y = intval(mxGetPr(paramImageSizeXYN)[1]);
    if ((mxGetNumberOfElements(paramImageSizeXYN) == 3) && (intval(mxGetPr(paramImageSizeXYN)[2])) > 1)
    {
        Camera->dim_N = intval(mxGetPr(paramImageSizeXYN)[2]);
    }
    else
    {
        Camera->dim_N = 1;
    }
    Camera->dim_pixel = pixel_size[intval(mxGetPr(paramPixelFormat)[0])-1];
    Camera->frames_count = Camera->dim_N * 2 + 1;
    
    if( mxGetPr(paramTimeout)[0] > 0 )
    {
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
        Camera->timeout_msec = (DWORD) (mxGetPr(paramTimeout)[0] * 1000);
#   else
        Camera->timeout_sec = floor(mxGetPr(paramTimeout)[0]);
        Camera->timeout_nsec = (mxGetPr(paramTimeout)[0] - floor(mxGetPr(paramTimeout)[0])) * 1000000000.0;
#   endif
    }
    
    /* Open the camera and configure it */
    if( (ss_PvCameraOpenByAddr(host, ePvAccessMaster, &(Camera->Handle)) != ePvErrSuccess) || \
        (Camera->Handle == NULL) )
    {
        Camera->Handle = NULL;
        ssSetErrorStatus(S,"Invalid camera IP or host address.");
        return;
    }
    
    if( (ss_PvAttrUint32Set(Camera->Handle, "Width",  Camera->dim_x) != ePvErrSuccess)       || \
        (ss_PvAttrUint32Set(Camera->Handle, "Height", Camera->dim_y) != ePvErrSuccess)       || \
        (ss_PvAttrEnumSet(Camera->Handle, "PixelFormat", pixel_format[intval(mxGetPr(paramPixelFormat)[0])-1]) != ePvErrSuccess) || \
        (ss_PvAttrEnumSet(Camera->Handle, "AcquisitionMode", "Continuous") != ePvErrSuccess) )
    {
        ssSetErrorStatus(S,"Can not set the camera attributes.");
        return;
    }
    
    if( intval(mxGetPr(paramSoftwareTrigger)[0]) )
    if( ss_PvAttrEnumSet(Camera->Handle, "FrameStartTriggerMode", "Software") != ePvErrSuccess )
    {
        ssSetErrorStatus(S,"Can not set the camera attributes.");
        return;
    }
    
    FrameSize = 0;
    ss_PvAttrUint32Get(Camera->Handle, "TotalBytesPerFrame", &FrameSize);
    if( FrameSize != Camera->dim_x * Camera->dim_y * Camera->dim_pixel )
    {
        ssSetErrorStatus(S,"Wrong frame size was read from the camera.");
        return;
    }
    
    /* Allocate memory for the image buffers */
    Camera->Frames = (tPvFrame*) calloc(Camera->frames_count, sizeof(tPvFrame));
    Camera->frames_queue = (tPvFrame**) calloc(Camera->frames_count+1, sizeof(tPvFrame*));
    for( i=0; i<Camera->frames_count; i++ )
    {
        Camera->Frames[i].ImageBuffer = calloc(1, FrameSize);
        Camera->Frames[i].ImageBufferSize = FrameSize;
        Camera->Frames[i].Context[0] = Camera;
    }
    
    /* Start the capture */
    if (ss_PvCaptureStart(Camera->Handle) != ePvErrSuccess)
    {
        ssSetErrorStatus(S,"Can not start the camera capture.");
        return;
    }
    
    if( ss_PvCommandRun(Camera->Handle, "AcquisitionStart") != ePvErrSuccess )
    {
        ssSetErrorStatus(S,"Can not start the camera acquisition.");
        return;
    }
    
    /* Queue the image buffers */
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    EnterCriticalSection( &(Camera->csec) );
#   else
    pthread_mutex_lock( &(Camera->mutex) );
#   endif
    for( i=0; i<Camera->frames_count-1; i++ )
    {
        ss_PvCaptureQueueFrame(Camera->Handle, &(Camera->Frames[i]), callback_fn);
    }
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    LeaveCriticalSection( &(Camera->csec) );
#   else
    pthread_mutex_unlock( &(Camera->mutex) );
#   endif
    
    /* Software trigger */
    if( intval(mxGetPr(paramSoftwareTrigger)[0]) )
    {
        ss_PvCommandRun(Camera->Handle, "FrameStartTriggerSoftware");
    }
}


/* Function: mdlOutputs =======================================================
 *
 */
#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* Retrieve C object from the pointers vector */
    tCamera* Camera = (tCamera*) ssGetPWork(S)[0];
    uint8_T *py0;
    int_T y_len;
    int frames_idx_in;
    int frames_idx_out;
    int frames_idx_out2;
    int frames_avail;
    int frames_avail2;
    int frames_ignore;
    int frames_reject;
    tPvFrame* Frame;
    unsigned long idx_x;
    unsigned long idx_y;
    unsigned long color_plane_size;
    unsigned char* buf;
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    DWORD rc;
#   else
    struct timespec timeout;
    int rc;
#   endif
    
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }

    py0 = (uint8_T *)ssGetOutputPortSignal(S,0);
    y_len = ssGetOutputPortWidth(S, 0);
    
    /* Suspend thread and wait for N frames */
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    if( Camera->timeout_msec )
    {
        rc = WaitForSingleObject( Camera->event, Camera->timeout_msec );
        if( rc != WAIT_OBJECT_0 )
        {
            ssSetErrorStatus(S,"Timeout occured.");
            return;
        }
    }
    EnterCriticalSection( &(Camera->csec) );
    frames_idx_in = Camera->frames_idx_in;
    frames_idx_out = Camera->frames_idx_out;
    LeaveCriticalSection( &(Camera->csec) );
#   else
    if( Camera->timeout_sec || Camera->timeout_nsec )
    {
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec  += Camera->timeout_sec;
        timeout.tv_nsec += Camera->timeout_nsec;
        if( timeout.tv_nsec >= 1000000000l )
        {
            timeout.tv_nsec -= 1000000000l;
            timeout.tv_sec++;
        }

        rc = 0;
        pthread_mutex_lock(&(Camera->mutex));
        while (!Camera->Avail && rc==0)
        {
            rc = pthread_cond_timedwait(&(Camera->cond), &(Camera->mutex), &timeout);
        }
        if( rc == ETIMEDOUT )
        {
            ssSetErrorStatus(S,"Timeout occured.");
            return;
        }
        Camera->Avail = false;
        frames_idx_in = Camera->frames_idx_in;
        frames_idx_out = Camera->frames_idx_out;
        pthread_mutex_unlock(&(Camera->mutex));
    }
    else
    {
        pthread_mutex_lock(&(Camera->mutex));
        frames_idx_in = Camera->frames_idx_in;
        frames_idx_out = Camera->frames_idx_out;
        pthread_mutex_unlock(&(Camera->mutex));
    }
#   endif
    
    frames_avail = frames_idx_in - frames_idx_out;
    if( frames_avail < 0 )
    {
        frames_avail += Camera->frames_count + 1;
    }
    frames_ignore = ( frames_avail > (int)Camera->dim_N ? frames_avail-Camera->dim_N : 0 );
    frames_reject = 0;
    
    frames_avail2 = frames_avail;
    frames_idx_out2 = Camera->frames_idx_out;
    while( frames_avail2 )
    {
        Frame = Camera->frames_queue[frames_idx_out2];
        if( Frame->Status != ePvErrSuccess )
        {
            frames_reject++;
        }
        frames_avail2--;
    }

    frames_avail2 = frames_avail - frames_reject - frames_ignore;
    *((uint32_T *)ssGetOutputPortSignal(S,1)) = frames_avail2;
    
    /* Software trigger */
    if( intval(mxGetPr(paramSoftwareTrigger)[0]) )
    {
        ss_PvCommandRun(Camera->Handle, "FrameStartTriggerSoftware");
    }
    
    frames_avail2 = frames_avail;
    while( frames_avail2 )
    {
        Frame = Camera->frames_queue[Camera->frames_idx_out];
        if( Frame->Status == ePvErrSuccess )
        {
            if( frames_ignore )
            {
                frames_ignore--;
            }
            else
            {
                if( Camera->dim_pixel == 1)
                {
                    for( idx_x = 0; idx_x < Camera->dim_x; idx_x++ )
                    {
                        buf = (unsigned char*)Frame->ImageBuffer + idx_x;
                        for( idx_y = 0; idx_y < Camera->dim_y; idx_y++ )
                        {
                            *py0++ = *buf;
                            buf += Camera->dim_x;
                        }
                    }
                }
                else if( Camera->dim_pixel == 3)
                {
                    color_plane_size = Camera->dim_x * Camera->dim_y;
                    for( idx_x = 0; idx_x < Camera->dim_x; idx_x++ )
                    {
                        buf = (unsigned char*)Frame->ImageBuffer + idx_x*3;
                        for( idx_y = 0; idx_y < Camera->dim_y; idx_y++ )
                        {
                            *py0 = *buf;
                            *(py0+color_plane_size) = *(buf+1);
                            *(py0++ +2*color_plane_size) = *(buf+2);
                            buf += Camera->dim_x*3;
                        }
                    }
                }
            }
        }
#       if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
        EnterCriticalSection( &(Camera->csec) );
#       else
        pthread_mutex_lock( &(Camera->mutex) );
#       endif
        if( ++Camera->frames_idx_out >= Camera->frames_count+1 )
        {
            Camera->frames_idx_out = 0;
        }
#       if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
        LeaveCriticalSection( &(Camera->csec) );
#       else
        pthread_mutex_unlock( &(Camera->mutex) );
#       endif
        ss_PvCaptureQueueFrame(Camera->Handle, Frame, callback_fn);
        frames_avail2--;
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
    UNUSED_ARG(tid); /* not used in single tasking mode */
    
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
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
    tCamera* Camera;
    unsigned int i;

    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }

    /* Retrieve and destroy C object */
    Camera = (tCamera*)ssGetPWork(S)[0];
    
    num_instances--;
    
    if( Camera )
    {
        if( Camera->Handle )
#       ifdef LOAD_PVLIB_AT_RUNTIME
        if( PvLib )
#       endif
        {
            ss_PvCommandRun(Camera->Handle,"AcquisitionStop");
            ss_PvCaptureEnd(Camera->Handle);
            ss_PvCaptureQueueClear(Camera->Handle);
            ss_PvCameraClose(Camera->Handle);
        }

        if( num_instances == 0 )
        {
#           if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
            if( Camera->csec_init ) DeleteCriticalSection( &(Camera->csec) );
            if( Camera->event ) CloseHandle( Camera->event );
#           else
            if( Camera->mutex_init ) pthread_mutex_destroy( &(Camera->mutex) );
            if( Camera->cond_init )  pthread_cond_destroy( &(Camera->cond) );
#           endif
        }
        
        if( Camera->Frames )
        {
            for( i=0; i<Camera->frames_count; i++ )
            {
                if( Camera->Frames[i].ImageBuffer ) free(Camera->Frames[i].ImageBuffer);
            }
            free(Camera->Frames);
        }
        if( Camera->frames_queue ) free(Camera->frames_queue);
        free(Camera);
        ssGetPWork(S)[0] = NULL;
    }
    
    if( num_instances == 0 )
    {
#       ifdef LOAD_PVLIB_AT_RUNTIME
        if( PvLib )
        {
            ss_PvUnInitialize();
            close_library(PvLib);
            PvLib = NULL;
        }
#       else
        ss_PvUnInitialize();
#       endif
#       if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
        WSACleanup();
#       endif
    }
}


static void PVDECL callback_fn(tPvFrame* Frame)
{
    tCamera* Camera = (tCamera*) Frame->Context[0];
    int frames_avail;
    int frames_avail2;
    int frames_ignore;
    int frames_reject;
    
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    EnterCriticalSection( &(Camera->csec) );
#   else
    pthread_mutex_lock( &(Camera->mutex) );
#   endif
    
    Camera->frames_queue[Camera->frames_idx_in++] = Frame;
    if( Camera->frames_idx_in >= Camera->frames_count+1 )
    {
        Camera->frames_idx_in = 0;
    }
    
    /* Check condition for unblocking the Simulink thread */
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    if( Camera->timeout_msec )
#   else
    if( Camera->timeout_sec || Camera->timeout_nsec )
#   endif
    {
        frames_avail = Camera->frames_idx_in - Camera->frames_idx_out;
        if( frames_avail < 0 )
        {
            frames_avail += Camera->frames_count + 1;
        }
        frames_ignore = ( frames_avail > (int) Camera->dim_N ? frames_avail-Camera->dim_N : 0 );
        frames_reject = 0;

        frames_avail2 = frames_avail;
        while( frames_avail2 )
        {
            if( Camera->frames_queue[Camera->frames_idx_out]->Status != ePvErrSuccess )
            {
                frames_reject++;
            }
            frames_avail2--;
        }

        frames_avail2 = frames_avail - frames_reject - frames_ignore;
        if( frames_avail2 >= (int) Camera->dim_N )
        {
#           if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
            SetEvent( Camera->event );
#           else
            Camera->Avail = true;
            pthread_cond_broadcast( &(Camera->cond) );
#           endif
        }
    }
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    LeaveCriticalSection( &(Camera->csec) );
#   else
    pthread_mutex_unlock( &(Camera->mutex) );
#   endif
}

#ifdef LOAD_PVLIB_AT_RUNTIME
static lib_handle load_library(char* errmsg)
{
#   if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    const char *PvLibName = "PvAPI.dll";
#   else
    const char *PvLibName = "libPvAPI.so";
#   endif
    lib_handle lib;
    union tPvLibFnConv conv;

    errmsg[0] = 0;

#   if defined(_WIN32) || defined (_WIN64) || defined(__LCC__)
    lib = LoadLibrary(PvLibName);
#   else
    lib = dlopen(PvLibName, RTLD_NOW | RTLD_LOCAL);
#   endif

    if(lib == NULL)
    {
        sprintf(errmsg, "Could not open library: %s", PvLibName);
    }
    else
    {
        LOAD_LIB_FN(ss_PvInitializeNoDiscovery, conv, lib, "PvInitializeNoDiscovery");
        LOAD_LIB_FN(ss_PvCameraOpenByAddr,      conv, lib, "PvCameraOpenByAddr");
        LOAD_LIB_FN(ss_PvCaptureStart,          conv, lib, "PvCaptureStart");
        LOAD_LIB_FN(ss_PvAttrUint32Set,         conv, lib, "PvAttrUint32Set");
        LOAD_LIB_FN(ss_PvAttrEnumSet,           conv, lib, "PvAttrEnumSet");
        LOAD_LIB_FN(ss_PvCommandRun,            conv, lib, "PvCommandRun");
        LOAD_LIB_FN(ss_PvAttrUint32Get,         conv, lib, "PvAttrUint32Get");
        LOAD_LIB_FN(ss_PvCaptureQueueFrame,     conv, lib, "PvCaptureQueueFrame");
        LOAD_LIB_FN(ss_PvCaptureEnd,            conv, lib, "PvCaptureEnd");
        LOAD_LIB_FN(ss_PvCaptureQueueClear,     conv, lib, "PvCaptureQueueClear");
        LOAD_LIB_FN(ss_PvCameraClose,           conv, lib, "PvCameraClose");
        LOAD_LIB_FN(ss_PvUnInitialize,          conv, lib, "PvUnInitialize");
    }

    return lib;
}

static void close_library(lib_handle PvLib)
{
    if(PvLib)
    {
#       if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
        FreeLibrary(PvLib);
#       else
        dlclose(PvLib);
#       endif
    }
}
#endif

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
