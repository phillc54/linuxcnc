#include "rtapi_json5.h"

int indent_level=0;

void rtapi_print_indent()
{
    // KLUDGE inefficient spamming for indents...
    for (int k=0; k<indent_level; ++k) {
        rtapi_print("  ");
    }

}
