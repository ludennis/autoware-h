#ifndef __TRACE_PRAGMA_H__
#define __TRACE_PRAGMA_H__

#if _MSC_VER
    #define TRACE_PRAGMA(directive) __pragma(directive)
#elif __GNUC__
    #define TRACE_PRAGMA(directive) _Pragma(#directive)
#endif

#if _MSC_VER
    #define TRACE_PRAGMA_MESSAGE(msg) TRACE_PRAGMA(message(msg))
    #define TRACE_PRAGMA_FILE_LINE_MESSAGE(msg) \
        TRACE_PRAGMA_MESSAGE(__TRACE_EXPAND(__FILE__) \
            "(" __TRACE_STRINGIFY2(__LINE__) "): " msg)
#elif __GNUC__
    #define TRACE_PRAGMA_MESSAGE(msg) TRACE_PRAGMA(message msg)
#endif

#endif // __TRACE_PRAGMA_H__
