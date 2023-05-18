#ifndef __TRACE_GL_H__
#define __TRACE_GL_H__

#if _DEBUG || TRACE_FORCE_ENABLE
    #define TRACE_GL_ERRORS \
        TRACE_GL_ERROR(GL_INVALID_ENUM) \
        TRACE_GL_ERROR(GL_INVALID_VALUE) \
        TRACE_GL_ERROR(GL_INVALID_OPERATION) \
        TRACE_GL_ERROR(GL_INVALID_FRAMEBUFFER_OPERATION) \
        TRACE_GL_ERROR(GL_OUT_OF_MEMORY) \

    #define TRACE_GL_ERROR(errorCode) \
        case errorCode: \
            TRACE_ERROR("%s failed with %s", funcNameString, #errorCode); \
            break;

    #define TRACE_GL(funcName, ...) \
        funcName(__VA_ARGS__); \
        TRACE_CHECK_LEVEL(LEVEL_DEBUG) { \
            const char * funcNameString = #funcName; \
            GLenum glError = glGetError(); \
            switch(glError) \
            { \
            case GL_NO_ERROR: \
                break; \
                TRACE_GL_ERRORS \
            default: \
                TRACE_ERROR("%s failed with error code 0x%x", \
                    funcNameString, glError); \
                break; \
            } \
        }
#else
    #define TRACE_GL(funcName, ...) funcName(__VA_ARGS__)
#endif

#endif // __TRACE_GL_H__
