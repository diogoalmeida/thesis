#ifndef __cd_signals_H
#define __cd_signals_H
#include "tmwtypes.h"


/* Check compile environment */
#if defined(_WIN32) || defined(_WIN64) || defined(__LCC__)
#define ENV_WINDOWS
#if defined(_WIN64)
#define ENV_64_BIT
#else
#define ENV_32_BIT
#endif
#else
#define ENV_LINUX
#if defined(__x86_64__) || defined(__ppc64__)
#define ENV_64_BIT
#else
#define ENV_32_BIT
#endif
#endif


typedef struct
{
    int_T                   size;
    int_T                   continuous;
    real_T                  **sig_disc;
    real_T                  *sig_cont;
} CD_SIGNALS_SHARED_SIG;

typedef struct CD_SIGNALS_SHARED_NODE CD_SIGNALS_SHARED_NODE;
struct CD_SIGNALS_SHARED_NODE
{
    char_T                  block_name[33];
    int_T                   num_inputs;
    int_T                   num_outputs;
    CD_SIGNALS_SHARED_SIG   *list_inputs;
    CD_SIGNALS_SHARED_SIG   *list_outputs;
    CD_SIGNALS_SHARED_NODE  *next_node;
};

boolean_T cd_write_signal( uint32_T *node_id, int_T index, const real_T *src, int_T size);
boolean_T cd_read_signal( uint32_T *node_id, int_T index, real_T *dest, int_T size);


#endif
