/* *******************************************************************
   **** http://www.UAVdesign.net/                                 ****
   **** To build this mex function use:                           ****
   **** mex 'threadpriority.c' -lrt                               ****
   **** add/modify                                                ****
   ****    LIBS = -lrt to the library section in ert_unix.tmf     ****
   ******************************************************************* */


#define S_FUNCTION_NAME threadpriority
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
#include <windows.h>

#else
#include <sys/resource.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <errno.h>
#define CLOCK_MONOTONIC			1
#endif

typedef struct
{
	int                num_instances;
    int                supervisor_created;
    int                policy_changed;
    int                request_stop;
    int                activity;
    int                error;
#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
    Not implemented for Windows yet
#else
    pthread_t          simulink_thread;
    pthread_t          supervisor_thread;
    pthread_mutex_t    mutex;
    pthread_cond_t     cond;
    struct sched_param old_param;
    int                old_policy;
    int                old_nice;
#endif
} ThreadMonitor, *pThreadMonitor;

static ThreadMonitor TM;
static void *supervisor_threadfunc(void *myarg);

#define NPARAMS                 8
#define paramPolicySimWin       ssGetSFcnParam(S,0)
#define paramPolicyExtWin       ssGetSFcnParam(S,1)
#define paramPolicySimLinux     ssGetSFcnParam(S,2)
#define paramPolicyExtLinux     ssGetSFcnParam(S,3)
#define paramPrioritySimWin     ssGetSFcnParam(S,4)
#define paramPriorityExtWin     ssGetSFcnParam(S,5)
#define paramPrioritySimLinux   ssGetSFcnParam(S,6)
#define paramPriorityExtLinux   ssGetSFcnParam(S,7)

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
    int i;
    int r_min[8]={1, 1, 1, 1, 0, 0, 1, 1};
    int r_max[8]={2, 2, 3, 3, 10, 10, 98, 98};
    for( i=0; i<8; i++ )
    {
        if ( (!IS_PARAM_DOUBLE(ssGetSFcnParam(S,i))) || (mxGetNumberOfElements(ssGetSFcnParam(S,i)) != 1) )
        {
            ssSetErrorStatus(S, "The policies and the priorities have to be integer numbers.");
            return;
        }
    }
    if( intval(mxGetPr(paramPolicySimLinux)[0]) == 1 )
    {
        r_min[6] = -20;
        r_max[6] =  19;
    }
    if( intval(mxGetPr(paramPolicyExtLinux)[0]) == 1 )
    {
        r_min[7] = -20;
        r_max[7] =  19;
    }
    for( i=0; i<8; i++ )
    {
        if ( (intval(mxGetPr(ssGetSFcnParam(S,i))[0]) < r_min[i]) || (intval(mxGetPr(ssGetSFcnParam(S,i))[0]) > r_max[i]) )
        {
            ssSetErrorStatus(S, "One of the parameters is out of range.");
            return;
        }
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
    pthread_condattr_t cond_attr;
    int error;
    struct sched_param param;  /* scheduling priority */
    int policy;                /* policy */
    const int policy_tbl[3] = {SCHED_OTHER, SCHED_FIFO, SCHED_RR};
    int param_policy;
    int param_priority;
    
    if( TM.num_instances )
    {
        ssSetErrorStatus(S, "You can have only one instance of the threadpriority block in your model.");
        return;
    }
    else
    {
        TM.num_instances = 1;
    }
    
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        return;
    }
    
    /* Get the current thread id */
    TM.simulink_thread = pthread_self();
    
    /* Get the scheduling priority of the thread */
    pthread_getschedparam(TM.simulink_thread, &TM.old_policy, &TM.old_param);
    if( TM.old_policy == SCHED_OTHER )
    {
        TM.old_nice = getpriority( PRIO_PROCESS, 0 );
    }
    
    /* create the mutex and the condition variable */
    if( pthread_mutex_init(&TM.mutex,NULL) )
    {
        ssSetErrorStatus(S, "Can not initialize supervisor thread mutex.");
        return;
    }
    pthread_condattr_init( &cond_attr );
    pthread_condattr_setclock( &cond_attr, CLOCK_MONOTONIC );
    error = pthread_cond_init(&TM.cond,&cond_attr);
    pthread_condattr_destroy( &cond_attr );
    if( error )
    {
        ssSetErrorStatus(S, "Can not initialize supervisor thread condition.");
        return;
    }
    
    /* create the supervisor thread */
    if( pthread_create(&TM.supervisor_thread, NULL, supervisor_threadfunc, (void *) &TM) )
    {
        ssSetErrorStatus(S, "Can not create thread priority supervisor thread.");
        return;
    }
    TM.supervisor_created = 1;

    /* Set the new scheduling priority of the threads */
#if defined(MATLAB_MEX_FILE)
    param_policy = intval(mxGetPr(paramPolicySimLinux)[0]);
    param_priority = intval(mxGetPr(paramPrioritySimLinux)[0]);
#else
    param_policy = intval(mxGetPr(paramPolicyExtLinux)[0]);
    param_priority = intval(mxGetPr(paramPriorityExtLinux)[0]);
#endif
    policy = SCHED_RR;
    if( param_policy == 1 )
    {
        param.sched_priority = 1;
    }
    else
    {
        param.sched_priority = param_priority + 1;
    }
    pthread_setschedparam(TM.supervisor_thread, policy, &param);

    if( param_policy == 1 )
    {
        setpriority( PRIO_PROCESS, 0, param_priority );
    }
    else
    {
        policy = policy_tbl[param_policy-1];
        param.sched_priority = param_priority;
        pthread_setschedparam(TM.simulink_thread, policy, &param);
    }
    
    TM.policy_changed = 1;
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
    
    if( TM.supervisor_created )
    {
        pthread_mutex_lock(&TM.mutex);
        TM.activity = 1;
        pthread_mutex_unlock(&TM.mutex);
        pthread_cond_broadcast(&TM.cond);
    }
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
    if( ssGetSimMode(S) != SS_SIMMODE_NORMAL )
    {
        memset( &TM, 0, sizeof(TM) );
        return;
    }
    
    /* set the priority back to normal */
    if( TM.policy_changed )
    {
        if( TM.old_policy == SCHED_OTHER )
        {
            setpriority( PRIO_PROCESS, 0, TM.old_nice );
        }
        else
        {
            pthread_setschedparam(TM.simulink_thread, TM.old_policy, &TM.old_param);
        }
    }
    
    /* stop the supervisor thread and join */
    if( TM.supervisor_created )
    {
        pthread_mutex_lock(&TM.mutex);
        TM.request_stop = 1;
        pthread_mutex_unlock(&TM.mutex);
        pthread_cond_broadcast(&TM.cond);
        pthread_join(TM.supervisor_thread,NULL);
    }
    
    pthread_mutex_destroy( &TM.mutex );
    pthread_cond_destroy( &TM.cond );
    memset( &TM, 0, sizeof(TM) );
}


static void *supervisor_threadfunc(void *myarg)
{
    pThreadMonitor pTM;
    struct timespec abstime;
    int activity;
    int error;
    
    pTM=(pThreadMonitor) myarg;
    
    pthread_mutex_lock( &(pTM->mutex) );
    
    clock_gettime( CLOCK_MONOTONIC, &abstime );
    abstime.tv_sec += 1;
    
    while( !pTM->request_stop )
    {
        error = pthread_cond_timedwait( &(pTM->cond), &(pTM->mutex), &abstime );
        if( pTM->request_stop ) break;
        
        /* do something here while holding the mutex */
        activity = pTM->activity;
        pTM->activity = 0;
        if( error == ETIMEDOUT )
        {
            pTM->error = 1;
        }
        pthread_mutex_unlock( &(pTM->mutex) );
        
        /* perform processing */
        if( error == ETIMEDOUT )
        {
            /* set the priority back to normal */
            if( pTM->policy_changed )
            {
                if( pTM->old_policy == SCHED_OTHER )
                {
                    setpriority( PRIO_PROCESS, 0, pTM->old_nice );
                }
                else
                {
                    pthread_setschedparam( pTM->simulink_thread, pTM->old_policy, &(pTM->old_param) );
                }
            }
            break;
        }
        else if( activity )
        {
            clock_gettime( CLOCK_MONOTONIC, &abstime );
            abstime.tv_sec += 1;
        }
        
        pthread_mutex_lock( &(pTM->mutex) );
    }
    pthread_mutex_unlock( &(pTM->mutex) );
    return NULL;
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
