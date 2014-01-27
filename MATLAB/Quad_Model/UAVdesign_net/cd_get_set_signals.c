/* *******************************************************************
   **** http://www.UAVdesign.net/                                 ****
   **** To build this mex function use:                           ****
   **** mex 'cd_get_set_signals.c'                                ****
   ******************************************************************* */

#define S_FUNCTION_NAME cd_get_set_signals
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "cd_signals.h"

#define NPARAMS                       4
#define paramBlockName                ssGetSFcnParam(S,0)
#define paramMode                     ssGetSFcnParam(S,1)
#define paramPortsVector              ssGetSFcnParam(S,2)
#define paramSampleTime               ssGetSFcnParam(S,3)


#define MODE_SET_INPUTS               1
#define MODE_GET_OUTPUTS              2
#define MODE_SIGNAL_ID                3

static CD_SIGNALS_SHARED_NODE* cd_signals_root = NULL;
static CD_SIGNALS_SHARED_NODE* cd_add_shared_node(const char_T *block_name, int_T num_inputs, int_T num_outputs);
static CD_SIGNALS_SHARED_SIG*  cd_set_shared_sig(CD_SIGNALS_SHARED_NODE* node, int_T inout, int_T index, int_T size, int_T continuous, void *sig_addr);
static void cd_free_all_nodes( void );

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
    int_T param;
    real_T* list;
    
    for( param=1; param<NPARAMS; param++ )
    {
        if ( !IS_PARAM_DOUBLE(ssGetSFcnParam(S, param)) && (mxGetNumberOfElements(ssGetSFcnParam(S, param)) < 1) )
        {
            ssSetErrorStatus(S, "Parameter is not a double.");
            return;
        }
    }
    
    if ( !mxIsChar(paramBlockName) || (mxGetNumberOfElements(paramBlockName) < 1) || (mxGetNumberOfElements(paramBlockName) > 32) )
    {
        ssSetErrorStatus(S, "The Block Name parameter must be a nonempty string with at most 32 characters.");
        return;
    }
    
    if( intval(mxGetPr(paramMode)[0]) != MODE_SIGNAL_ID )
    {
        if ( mxGetNumberOfElements(paramPortsVector) < 1 )
        {
            ssSetErrorStatus(S, "The ports vector must have at least one component.");
            return;
        }

        list = mxGetPr(paramPortsVector);
        if( list == NULL )
        {
            ssSetErrorStatus(S, "The ports vector must have integer components.");
            return;
        }

        for ( param=0; param<mxGetNumberOfElements(paramPortsVector); param++ )
        {
            if( intval(list[param]) < 1 )
            {
                ssSetErrorStatus(S, "The ports vector must have positive components.");
            }
        }
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
# define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{
    int_T nports;
    const real_T* list;
    int_T i;
    
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S))
    {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
	} 
    else return; /* Parameter mismatch will be reported by Simulink */
#endif

    for( i=0; i<NPARAMS; i++ )
    {
        ssSetSFcnParamTunable(S,i,false);
    }

    ssSetNumSampleTimes(S, 1);
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumRWork(S, 0);
    ssSetNumDWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    switch( intval(mxGetPr(paramMode)[0]) )
    {
        case MODE_SET_INPUTS:
            ssSetNumPWork(S, 0);
            ssSetNumIWork(S, 0);
            nports = mxGetNumberOfElements(paramPortsVector);
            list = mxGetPr(paramPortsVector);
            if( !ssSetNumInputPorts(S, 0) ) return;
            if( !ssSetNumOutputPorts(S, nports) ) return;
            for( i=0; i<nports; i++ )
            {
                ssSetOutputPortWidth(S, i, intval(list[i]));
                ssSetOutputPortDataType(S, i, SS_DOUBLE);
                ssSetOutputPortOptimOpts(S, i, SS_NOT_REUSABLE_AND_GLOBAL);
            }
            break;
        case MODE_GET_OUTPUTS:
            ssSetNumPWork(S, 0);
            ssSetNumIWork(S, 0);
            nports = mxGetNumberOfElements(paramPortsVector);
            list = mxGetPr(paramPortsVector);
            if( !ssSetNumInputPorts(S, nports) ) return;
            if( !ssSetNumOutputPorts(S, 0) ) return;
            for( i=0; i<nports; i++ )
            {
                ssSetInputPortWidth(S, i, intval(list[i]));
                ssSetInputPortDataType(S, i, SS_DOUBLE );
                ssSetInputPortComplexSignal(S, i, 0);
                ssSetInputPortDirectFeedThrough(S, i, 0);
                ssSetInputPortRequiredContiguous(S, i, 1); /*direct input signal access*/
                ssSetInputPortOptimOpts(S, i, SS_NOT_REUSABLE_AND_GLOBAL);
                ssSetInputPortOverWritable(S, i, 0);
            }
            break;
        case MODE_SIGNAL_ID:
            ssSetNumPWork(S, 1);
            ssSetNumIWork(S, 1);
            if( !ssSetNumInputPorts(S, 0) ) return;
            if( !ssSetNumOutputPorts(S, 1) ) return;
#ifdef ENV_64_BIT
            ssSetOutputPortWidth(S, 0, 2);
#else
            ssSetOutputPortWidth(S, 0, 1);
#endif
            ssSetOutputPortDataType(S, 0, SS_UINT32);
            break;
        default:
            ssSetErrorStatus(S,"The mode has to be 1, 2 or 3.");
            return;
    }
        
    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_CALL_TERMINATE_ON_EXIT);
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
    char_T* block_name;
    int_T block_name_len;
    int_T nports;
    int_T i;
    CD_SIGNALS_SHARED_NODE* node;
    
    block_name_len = mxGetM(paramBlockName) * mxGetN(paramBlockName) + 1;
	block_name = (char_T*) malloc(block_name_len);
    mxGetString(paramBlockName, block_name, block_name_len);
            
    switch( intval(mxGetPr(paramMode)[0]) )
    {
        case MODE_SET_INPUTS:
            nports = ssGetNumOutputPorts(S);
            node = cd_add_shared_node(block_name, nports, 0);
            free( block_name );
            if( node == NULL )
            {
                ssSetErrorStatus(S,"The block name is already used elsewhere specifying an input.");
                return;
            }
            for( i=0; i<nports; i++ )
            {
                cd_set_shared_sig(node, 0, i, ssGetOutputPortWidth(S, i), 1, ssGetOutputPortRealSignal(S, i));
            }
            break;
        case MODE_GET_OUTPUTS:
            nports = ssGetNumInputPorts(S);
            node = cd_add_shared_node(block_name, 0, nports);
            free( block_name );
            if( node == NULL )
            {
                ssSetErrorStatus(S,"The block name is already used elsewhere specifying an output.");
                return;
            }
            for( i=0; i<nports; i++ )
            {
                cd_set_shared_sig(node, 1, i, ssGetInputPortWidth(S, i), 1, (void *)ssGetInputPortRealSignal(S, i));
            }
            break;
        case MODE_SIGNAL_ID:
            node = cd_add_shared_node(block_name, 0, 0);
            free( block_name );
            if( node == NULL )
            {
                ssSetErrorStatus(S,"Error getting the node ID.");
                return;
            }
            ssGetPWork(S)[0] = node;
            break;
        default:
            free( block_name );
            {
                ssSetErrorStatus(S,"Wrong value for the Mode parameter.");
                return;
            }
    }
}


/* Function: mdlOutputs =======================================================
 *
 */
#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
{
    CD_SIGNALS_SHARED_NODE* node;
    uint32_T* out;
    
    if( (intval(mxGetPr(paramMode)[0]) == MODE_SIGNAL_ID) && (ssGetIWork(S)[0] == 0) )
    {
        ssGetIWork(S)[0] = 1;
        node = (CD_SIGNALS_SHARED_NODE*)ssGetPWork(S)[0];
        if( (node->num_inputs == 0) && (node->num_outputs == 0) )
        {
            ssSetErrorStatus(S,"The block name does not correspond to any remote input or output block in the model.");
            return;
        }
        out = (uint32_T*)ssGetOutputPortSignal(S, 0);
#ifdef ENV_64_BIT
		out[0] = (uint64_T)(ssGetPWork(S)[0]);
        out[1] = ((uint64_T)(ssGetPWork(S)[0]))>>32;
#else
        out[0] = (uint32_T)(ssGetPWork(S)[0]);
#endif
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
    cd_free_all_nodes();
}

static CD_SIGNALS_SHARED_NODE* cd_add_shared_node(const char_T *block_name, int_T num_inputs, int_T num_outputs)
{
    CD_SIGNALS_SHARED_NODE* node      = cd_signals_root;
    CD_SIGNALS_SHARED_NODE* last_node = NULL;
    int_T already_exists = 0;
    while( node )
    {
        last_node = node;
        if( strcmp(node->block_name, block_name) == 0 )
        {
            /* A node with the same name already exists */
            if( node->num_inputs && num_inputs ) return NULL;
            if( node->num_outputs && num_outputs ) return NULL;
            already_exists = 1;
            break;
        }
        node = node->next_node;
    }
    
    if( !already_exists )
    {
        node = (CD_SIGNALS_SHARED_NODE*) calloc(1, sizeof(CD_SIGNALS_SHARED_NODE));
        strncpy(node->block_name, block_name, 32);
        if( last_node )
        {
            last_node->next_node = node;
        }
        else
        {
            cd_signals_root = node;
        }
        last_node = node;
    }
    if( num_inputs )
    {
        last_node->num_inputs = num_inputs;
        last_node->list_inputs = (CD_SIGNALS_SHARED_SIG*) calloc(num_inputs, sizeof(CD_SIGNALS_SHARED_SIG));
    }
    if( num_outputs )
    {
        last_node->num_outputs = num_outputs;
        last_node->list_outputs = (CD_SIGNALS_SHARED_SIG*) calloc(num_outputs, sizeof(CD_SIGNALS_SHARED_SIG));
    }
    return last_node;
}

static CD_SIGNALS_SHARED_SIG*  cd_set_shared_sig(CD_SIGNALS_SHARED_NODE* node, int_T inout, int_T index, int_T size, int_T continuous, void *sig_addr)
{
    CD_SIGNALS_SHARED_SIG* list;
    int_T num_signals;
    
    list = (inout == 0 ? node->list_inputs : node->list_outputs);
    num_signals = (inout == 0 ? node->num_inputs : node->num_outputs);
    if( inout == 0 ) continuous = 1;
    
    if( (index < num_signals) && (list[index].size == 0) )
    {
        list[index].size = size;
        list[index].continuous = continuous;
        if( continuous )
        {
            list[index].sig_cont = sig_addr;
        }
        else
        {
            list[index].sig_disc = sig_addr;
        }
    }
    else return NULL;
    return &list[index];
}

static void cd_free_all_nodes( void )
{
    CD_SIGNALS_SHARED_NODE* node      = cd_signals_root;
    CD_SIGNALS_SHARED_NODE* next_node;
    cd_signals_root = NULL;
    while( node )
    {
        next_node = node->next_node;
        if( node->num_inputs ) free(node->list_inputs);
        if( node->num_outputs ) free(node->list_outputs);
        free(node);
        node = next_node;
    }
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
