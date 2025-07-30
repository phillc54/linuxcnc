#include "rtapi_json5.h"

int indent_level=0;

void rtapi_print_indent()
{
#ifdef JSON5_PRETTY_PRINT
    // KLUDGE inefficient spamming for indents...
    for (int k=0; k<indent_level; ++k) {
        rtapi_print("  ");
    }
#endif

}
