#ifndef GIGECOMM_H_INCLUDE
#define GIGECOMM_H_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
#include <windows.h>
#define PVDECL __stdcall
#else
#include <pthread.h>
#ifndef _LINUX
#define _LINUX
#endif
#endif

#define _x86
#include "PvApi.h"

typedef struct
{
    unsigned int     dim_x;
    unsigned int     dim_y;
    unsigned int     dim_N;
    unsigned int     dim_pixel;
    unsigned int     frames_idx_in;
    unsigned int     frames_idx_out;
    unsigned int     frames_count;
    tPvFrame**       frames_queue;
    unsigned long    UID;
    tPvHandle        Handle;
    tPvFrame*        Frames;
    bool             Stop;
#   if defined(_WIN32) || defined(_WIN64)  || defined(__LCC__)
    CRITICAL_SECTION csec;
    HANDLE           event;
    char             csec_init;
    DWORD            timeout_msec;
#   else
    pthread_mutex_t  mutex;
    pthread_cond_t   cond;
    char             mutex_init;
    char             cond_init;
    long             timeout_sec;
    long             timeout_nsec;
    bool             Avail;
#   endif
} tCamera;

#ifdef LOAD_PVLIB_AT_RUNTIME
/* Dynamic Library functions */
typedef tPvErr (PVDECL *tPvInitializeNoDiscovery)(void);
typedef tPvErr (PVDECL *tPvCameraOpenByAddr)(unsigned long IpAddr, tPvAccessFlags AccessFlag, tPvHandle* pCamera);
typedef tPvErr (PVDECL *tPvCaptureStart)(tPvHandle Camera);
typedef tPvErr (PVDECL *tPvAttrUint32Set)(tPvHandle Camera, const char* Name, tPvUint32 Value);
typedef tPvErr (PVDECL *tPvAttrEnumSet)(tPvHandle Camera, const char* Name, const char* Value);
typedef tPvErr (PVDECL *tPvCommandRun)(tPvHandle Camera, const char* Name);
typedef tPvErr (PVDECL *tPvAttrUint32Get)(tPvHandle Camera, const char* Name, tPvUint32* pValue);
typedef tPvErr (PVDECL *tPvCaptureQueueFrame)(tPvHandle Camera, tPvFrame* pFrame, tPvFrameCallback Callback);
typedef tPvErr (PVDECL *tPvCaptureEnd)(tPvHandle Camera);
typedef tPvErr (PVDECL *tPvCaptureQueueClear)(tPvHandle Camera);
typedef tPvErr (PVDECL *tPvCameraClose)(tPvHandle Camera);
typedef void   (PVDECL *tPvUnInitialize)(void);

union tPvLibFnConv
{
    void*                    sym;
    tPvInitializeNoDiscovery ss_PvInitializeNoDiscovery;
    tPvCameraOpenByAddr      ss_PvCameraOpenByAddr;
    tPvCaptureStart          ss_PvCaptureStart;
    tPvAttrUint32Set         ss_PvAttrUint32Set;
    tPvAttrEnumSet           ss_PvAttrEnumSet;
    tPvCommandRun            ss_PvCommandRun;
    tPvAttrUint32Get         ss_PvAttrUint32Get;
    tPvCaptureQueueFrame     ss_PvCaptureQueueFrame;
    tPvCaptureEnd            ss_PvCaptureEnd;
    tPvCaptureQueueClear     ss_PvCaptureQueueClear;
    tPvCameraClose           ss_PvCameraClose;
    tPvUnInitialize          ss_PvUnInitialize;
};

#if defined(_WIN32) || defined(_WIN64)  || defined(__LCC__)
typedef HMODULE lib_handle;
#define LOAD_LIB_FN(ss_fn_var, converter_var, lib_handle, lib_fn_name) \
    do                                                                 \
    {                                                                  \
        converter_var.sym = GetProcAddress( lib_handle, lib_fn_name);  \
        if( converter_var.sym == NULL ) ssPrintf("Could not load function %s\r\n", lib_fn_name); \
        ss_fn_var = converter_var. ss_fn_var;                          \
    } while(0)
#else
typedef void*   lib_handle;
#define LOAD_LIB_FN(ss_fn_var, converter_var, lib_handle, lib_fn_name) \
    do                                                                 \
    {                                                                  \
        converter_var.sym = dlsym( lib_handle, lib_fn_name);           \
        ss_fn_var = converter_var. ss_fn_var;                          \
    } while(0)
#endif
#endif /* LOAD_PVLIB_AT_RUNTIME */

#ifdef __cplusplus
}
#endif

#endif /* GIGECOMM_H_INCLUDE */
