#ifndef TP_CALL_WRAPPERS_HH
#define TP_CALL_WRAPPERS_HH

#include "tp_enums.h"
#include "tp_debug.h"

#define CH_ERR(expect__, mycall__) \
do {\
    int expt = expect__; \
    tp_err_t res__ = (tp_err_t)mycall__; \
    if (expt != res__) { \
        tp_debug_error_print("%s failed with %d (expect %d) at %s:%d\n", #mycall__, res__, expt, __FUNCTION__, __LINE__); \
        return res__; \
    } \
} while (0)

#define CHP(mycall__) CH_ERR(TP_ERR_OK, mycall__)



#define CH_FATAL(expect__, mycall__) \
do {\
    int expt = expect__; \
    tp_err_t res__ = (tp_err_t)mycall__; \
    if (expt != res__) { \
        emcOperatorError("%s failed with %d (expect %d) at %s:%d\n", #mycall__, res__, expt, __FUNCTION__, __LINE__); \
        return res__; \
    } \
} while (0)

#define CHF( mycall__) CH_FATAL(TP_ERR_OK, mycall__)
#endif // TP_CALL_WRAPPERS_HH
