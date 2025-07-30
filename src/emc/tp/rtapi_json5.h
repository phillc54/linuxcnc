/**
 * @file rtapi_json5.h
 *
 * @author Robert W. Ellenberg
 * @copyright Copyright (c) 2019 All rights reserved. This project is released under the GPL v2 license.
 */

#ifndef RTAPI_JSON5_H
#define RTAPI_JSON5_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtapi.h"  /* printing functions */

#ifndef JSON5_PRETTY_PRINT
static const char line_brk = ' ';
#else
static const char line_brk = '\n';
#endif
extern int indent_level;

// Macro wrappers to stringify the variable name
// NOTE: callers can directly call the underlying print function if a custom field name is needed
#define print_json5_double(varname_) print_json5_double_(#varname_, (double)varname_)
#define print_json5_string(varname_) print_json5_string_(#varname_, varname_)
#define print_json5_long(varname_) print_json5_long_(#varname_, (long)varname_)
#define print_json5_bool(varname_) print_json5_bool_(#varname_, (bool)varname_)
#define print_json5_int(varname_) print_json5_int_(#varname_, (int)varname_)
#define print_json5_unsigned(varname_) print_json5_unsigned_(#varname_, (unsigned)varname_)

#define print_json5_double_field(base_, varname_) print_json5_double_(#varname_, (base_)->varname_)
#define print_json5_string_field(base_, varname_) print_json5_string_(#varname_, (base_)->varname_ ?:"NULL")
#define print_json5_int_field(base_, varname_) print_json5_int_(#varname_, (base_)->varname_)
#define print_json5_unsigned_field(base_, varname_) print_json5_unsigned_(#varname_, (base_)->varname_)
#define print_json5_long_field(base_, varname_) print_json5_long_(#varname_, (base_)->varname_)
#define print_json5_long_long_field(base_, varname_) print_json5_long_long_(#varname_, (base_)->varname_)

// The printf-style wrapper doesn't work for some reason so just kludge it with a macro
// Not very performant but this is only for debugging so it's not worth optimizing now
#define rtapi_print_with_indent(...) \
do { \
    rtapi_print_indent(); \
    rtapi_print(__VA_ARGS__); \
} while (0)

// Hacky way to do realtime-logging of
// KLUDGE shove these in the header to avoid linking issues

void rtapi_print_indent();

static inline void print_json5_field_end_()
{
    rtapi_print(",%c", line_brk);
}

static inline void print_json5_start_()
{
    rtapi_print_with_indent("{");
    ++indent_level;
}

static inline void print_json5_end_()
{
    --indent_level;
    rtapi_print_with_indent("}%c", '\n');

}

static inline void print_json5_fieldname_(const char *fname)
{
    rtapi_print_with_indent("%s: ", fname ?: "NULL");
}

static inline void print_json5_array_start_(const char *arr_name)
{
    if (arr_name) {
        rtapi_print_with_indent("%s: [", arr_name);
    } else {

        rtapi_print_with_indent("[");
    }
}

static inline void print_json5_array_end_()
{
    rtapi_print_with_indent("],%c", line_brk);
}

static inline void print_json5_object_start_(const char *fname)
{
    if (fname) {
        rtapi_print_with_indent("%s: {%c", fname, line_brk);
    } else {
        rtapi_print_with_indent("{%c", line_brk);
    }
    ++indent_level;
}

static inline void print_json5_object_end_()
{
    --indent_level;
    rtapi_print_with_indent("},%c", line_brk);
}

static inline void print_json5_double_(const char *varname, double value)
{
    rtapi_print_with_indent("%s: %0.17g,%c", varname ?: "NO_NAME", value, line_brk);
}

static inline void print_json5_bool_(const char *varname, double value)
{
    rtapi_print_with_indent("%s: %s,%c", varname ?: "NO_NAME", value ? "true" : "false", line_brk);
}

static inline void print_json5_int_(const char *varname, int value)
{
    rtapi_print_with_indent("%s: %d,%c", varname ?: "NO_NAME", value, line_brk);
}

static inline void print_json5_long_(const char *varname, long value)
{
    rtapi_print_with_indent("%s: %ld,%c", varname ?: "NO_NAME", value, line_brk);
}

static inline void print_json5_long_long_(const char *varname, long long value)
{
    rtapi_print_with_indent("%s: %lld,%c", varname ?: "NO_NAME", value, line_brk);
}

static inline void print_json5_unsigned_(const char *varname, unsigned value)
{
    rtapi_print_with_indent("%s: %u,%c", varname ?: "NO_NAME", value, line_brk);
}

static inline void print_json5_string_(const char *field_name, const char *value)
{
    
    rtapi_print_with_indent("%s: \"%s\",%c", field_name ?: "NULL", value ?: "NULL", line_brk);
}

#ifdef __cplusplus
}				/* matches extern "C" for C++ */
#endif

#endif // RTAPI_JSON5_H
