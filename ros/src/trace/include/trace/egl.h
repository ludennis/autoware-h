#ifndef __TRACE_EGL_H__
#define __TRACE_EGL_H__

#if _DEBUG || TRACE_FORCE_ENABLE
    #define TRACE_EGL_ERRORS(funcName) \
        TRACE_EGL_ERROR(funcName, EGL_NOT_INITIALIZED) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_ACCESS) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_ALLOC) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_ATTRIBUTE) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_CONFIG) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_CONTEXT) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_CURRENT_SURFACE) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_DISPLAY) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_MATCH) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_NATIVE_PIXMAP) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_NATIVE_WINDOW) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_PARAMETER) \
        TRACE_EGL_ERROR(funcName, EGL_BAD_SURFACE) \
        TRACE_EGL_ERROR(funcName, EGL_CONTEXT_LOST) \

    #define TRACE_EGL_ERROR(funcName, code) \
        case code: \
            TRACE_ERROR("%s() failed with error code: %s", \
                #funcName, #code); \
            break;

    #define TRACE_EGL(eglFunc, ...) \
        eglFunc(__VA_ARGS__); \
        TRACE_CHECK_LEVEL(LEVEL_DEBUG) { \
            EGLint eglError = eglGetError(); \
            switch (eglError) { \
                case EGL_SUCCESS: \
                    break; \
                TRACE_EGL_ERRORS(eglFunc) \
                default: \
                    TRACE_ERROR("%s() failed with unknown error " \
                        "code: 0x%x", #eglFunc, eglError); \
                    break; \
            } \
        }
#else
    #define TRACE_EGL(eglFunc, ...) eglFunc(__VA_ARGS__)
#endif

#endif // __TRACE_EGL_H__
