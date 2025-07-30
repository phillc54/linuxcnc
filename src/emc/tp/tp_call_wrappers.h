#ifndef TP_CALL_WRAPPERS_H
#define TP_CALL_WRAPPERS_H

#include "tp_enums.h"
#include "tp_debug.h"

#define CH_ERR(expect__, mycall__) \
do {\
    int expt = expect__; \
    int res = mycall__; \
    if (expt != res) { \
        tp_debug_error_print("%s failed with %d (expect %d) at %s:%d\n", #mycall__, res, expt, __FUNCTION__, __LINE__); \
        return res; \
    } \
} while (0)

#define CHP(mycall__) CH_ERR(TP_ERR_OK, mycall__)


#define CH_FATAL(tp, expect__, mycall__) \
do {\
    int expt = expect__; \
    int res = mycall__; \
    if (expt != res) { \
        tpStopWithError(tp, "%s failed with %d (expect %d) at %s:%d\n", #mycall__, res, expt, __FUNCTION__, __LINE__); \
        return res; \
    } \
} while (0)

#define CHF(tp__, mycall__) CH_FATAL(tp__, TP_ERR_OK, mycall__)
#endif // TP_CALL_WRAPPERS_H
