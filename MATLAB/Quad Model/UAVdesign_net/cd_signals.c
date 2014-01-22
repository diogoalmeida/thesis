/* *******************************************************************
   **** http://www.UAVdesign.net/                                 ****
   ******************************************************************* */

#include "cd_signals.h"
#include <string.h>
#include <stdio.h>

boolean_T cd_write_signal( uint32_T *node_id, int_T index, const real_T *src, int_T size)
{
    CD_SIGNALS_SHARED_NODE* node;
    CD_SIGNALS_SHARED_SIG*  sig;

#ifdef ENV_64_BIT
    uint64_T                id;
#else
    uint32_T                id;
#endif
 
 	if( node_id == NULL ) return false;
 
#ifdef ENV_64_BIT
    id = node_id[0] | (uint64_T)(node_id[1])<<32;
#else
    id = node_id[0];
#endif

	node = (CD_SIGNALS_SHARED_NODE*) id;
	
    if( index < node->num_inputs )
    {
        sig = &node->list_inputs[index];
        if( size != sig->size ) return false;
        memcpy(sig->sig_cont, src, size*sizeof(real_T));
        return true;
    }
    else
    {
        return false;
    }
}

boolean_T cd_read_signal( uint32_T *node_id, int_T index, real_T *dest, int_T size)
{
    CD_SIGNALS_SHARED_NODE* node;
    CD_SIGNALS_SHARED_SIG* sig;
    int_T i;
    
#ifdef ENV_64_BIT
    uint64_T                id;
#else
    uint32_T                id;
#endif
 
 	if( node_id == NULL ) return false;
 
#ifdef ENV_64_BIT
    id = node_id[0] | (uint64_T)(node_id[1])<<32;
#else
    id = node_id[0];
#endif
	
	node = (CD_SIGNALS_SHARED_NODE*) id;
	
	if( index < node->num_outputs )
    {
        sig = &node->list_outputs[index];
        if( size != sig->size ) return false;
        if( sig->continuous )
        {
            memcpy(dest, sig->sig_cont, size*sizeof(real_T));
        }
        else
        {
            for( i=0; i<size; i++ )
            {
                dest[i] = *(sig->sig_disc[i]);
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}
