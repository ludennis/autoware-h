#ifndef __TRACE_UTILS_H__
#define __TRACE_UTILS_H__

#include <stdexcept>
#include <string>
#include <trace/pragma.h>

#define __TRACE_STRINGIFY(val) #val
#define __TRACE_STRINGIFY2(val) __TRACE_STRINGIFY(val)
#define __TRACE_EXPAND(val) val
#define __TRACE_GCC_VERSION \
    (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)

#define TRACE_LOGD(tag, ...) \
    Trace::Log(Trace::LEVEL_DEBUG, tag, __VA_ARGS__)
#define TRACE_LOGI(tag, ...) \
    Trace::Log(Trace::LEVEL_INFO, tag, __VA_ARGS__)
#define TRACE_LOGW(tag, ...) \
    Trace::Log(Trace::LEVEL_WARNING, tag, __VA_ARGS__)
#define TRACE_LOGE(tag, ...) \
    Trace::Log(Trace::LEVEL_ERROR, tag, __VA_ARGS__)

#ifdef WIN32
    #define TRACE_DEBUG_BREAK() __debugbreak()
#else
    #define TRACE_DEBUG_BREAK()
#endif

#ifdef TRACE_STATIC_LEVEL
    #define TRACE_LEVEL(level) TRACE_PRAGMA_MESSAGE( \
        "Cannot use TRACE_LEVEL() because TRACE_STATIC_LEVEL defined")
    #define TRACE_CHECK_LEVEL(level) if (TRACE_STATIC_LEVEL >= Trace::level)
#else
    #define TRACE_LEVEL(level) Trace::SetLevel(level)
    #define TRACE_CHECK_LEVEL(level) if (Trace::GetLevel() >= Trace::level)
#endif

#if TRACE_ENABLED
    #define TRACE_MESSAGE(...) \
        { \
            TRACE_CHECK_LEVEL(LEVEL_INFO) \
                TRACE_LOGI(TRACE_TAG, __VA_ARGS__); \
        }

    #define TRACE_DEBUG(...) \
        { \
            TRACE_CHECK_LEVEL(LEVEL_DEBUG) \
                TRACE_LOGD(TRACE_TAG, __VA_ARGS__); \
        }

    #define TRACE_WARNING(...) \
        { \
            TRACE_CHECK_LEVEL(LEVEL_WARNING) \
                TRACE_LOGW(TRACE_TAG, __VA_ARGS__); \
        }

    #define TRACE_ERROR(...) \
        { \
            TRACE_CHECK_LEVEL(LEVEL_ERROR) { \
                TRACE_LOGE(TRACE_TAG, __VA_ARGS__); \
                TRACE_DEBUG_BREAK(); \
            } \
        }

    #define TRACE_ASSERT(expr) \
        { \
            TRACE_CHECK_LEVEL(LEVEL_DEBUG) { \
                if (!(expr)) \
                { \
                    TRACE_ERROR("%s(%d) : Assertion failed : %s", \
                        __FILE__, __LINE__, #expr); \
                } \
            } \
        }

    #define TRACE_ASSERT_THROW(expr) \
        { \
            TRACE_CHECK_LEVEL(LEVEL_DEBUG) { \
                if (!(expr)) \
                { \
                    TRACE_ERROR("%s(%d) : Assertion failed : %s", \
                        __FILE__, __LINE__, #expr); \
                    TRACE_THROW("%s(%d) : Assertion failed : %s", \
                        __FILE__, __LINE__, #expr); \
                } \
            } \
        }

    #define TRACE_FAIL() \
        { \
            TRACE_CHECK_LEVEL(LEVEL_DEBUG) { \
                TRACE_ERROR("%s(%d) : Failed", __FILE__, __LINE__); \
            } \
        }

    #define TRACE_FAIL_THROW() \
        TRACE_FAIL(); TRACE_THROW("%s(%d) : Failed", __FILE__, __LINE__)
#else
    #define TRACE_MESSAGE(...)
    #define TRACE_DEBUG(...)
    #define TRACE_WARNING(...)
    #define TRACE_ERROR(...)
    #define TRACE_ASSERT(expr)
    #define TRACE_ASSERT_THROW(expr)
    #define TRACE_FAIL()
    #define TRACE_FAIL_THROW()
#endif

#define TRACE_THROW(...) \
    throw std::runtime_error(Trace::Format(__VA_ARGS__))
#define TRACE_ERROR_THROW(...) \
    { \
        TRACE_ERROR(__VA_ARGS__); \
        TRACE_THROW(__VA_ARGS__); \
    }

#if _MSC_VER
    #define TRACE_TODO(msg) TRACE_PRAGMA_FILE_LINE_MESSAGE("TODO " msg)
    #define TRACE_FIXME(msg) TRACE_PRAGMA_FILE_LINE_MESSAGE("FIXME " msg)
#elif __GNUC__
    #define TRACE_TODO(msg) TRACE_PRAGMA_MESSAGE("TODO " msg)
    #define TRACE_FIXME(msg) TRACE_PRAGMA_MESSAGE("FIXME " msg)
#endif

#ifndef __FUNCTION_NAME__
    #ifdef WIN32   //WINDOWS
        #define __FUNCTION_NAME__   __FUNCTION__
    #else          //*NIX
        #define __FUNCTION_NAME__   __func__
    #endif
#endif

namespace Trace {

    enum Level
    {
        LEVEL_ZERO = 0,
        LEVEL_INFO,
        LEVEL_ERROR,
        LEVEL_WARNING,
        LEVEL_DEBUG,
        LEVEL_MAX,
    };

    std::string Format(const char * format, ...);
    void Log(Level, const char * tag, const char * format, ...);

#ifndef TRACE_STATIC_LEVEL
    void SetLevel(unsigned level);
    unsigned GetLevel();
#endif

} // namespace Trace

#endif // __TRACE_UTILS_H__
