#include <mutex>
#include "trace/utils.h"

#ifdef __ANDROID__
    #include <android/log.h>
    #include <cstdarg>
#elif _WIN32
    #include <cstdarg>
    #include <cstdio>
    #include <windows.h>
#elif __linux__
    #include <cstdarg>
    #include <cstdio>
#endif

namespace Trace {

#ifndef TRACE_STATIC_LEVEL
    static unsigned sLevel = 0;

    void SetLevel(unsigned level)
    {
        if (level < LEVEL_MAX)
            sLevel = level;
        else
            Log(LEVEL_ERROR, "Trace",
                "Undefined level: %d", level);
    }

    unsigned GetLevel()
    {
        return sLevel;
    }
#endif

    std::string Format(const char * format, ...)
    {
        char buffer[1024];
        va_list args;
        va_start(args, format);
        vsprintf(buffer, format, args);
        va_end(args);
        return buffer;
    }

    void Log(Level level, const char * tag, const char * format, ...)
    {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);

    #ifdef __ANDROID__
        int logPriority;
        switch (level)
        {
        case LEVEL_DEBUG:
            logPriority = ANDROID_LOG_DEBUG;
            break;
        case LEVEL_INFO:
            logPriority = ANDROID_LOG_INFO;
            break;
        case LEVEL_WARNING:
            logPriority = ANDROID_LOG_WARN;
            break;
        case LEVEL_ERROR:
            logPriority = ANDROID_LOG_ERROR;
            break;
        }

        va_list args;
        va_start( args, format );
        __android_log_vprint( logPriority, tag, format, args );
        va_end( args);
    #else

        #if _WIN32
            static HANDLE sConsole;
        #endif

        static bool initialized = false;
        if (!initialized)
        {
            #if _WIN32
                AllocConsole();
                freopen("CONOUT$", "w", stderr);
                sConsole = GetStdHandle(STD_ERROR_HANDLE);
            #else
                std::setbuf(stderr, NULL);
            #endif
            initialized = true;
        }

        #if _WIN32
            #define COLOR_GREY \
                FOREGROUND_INTENSITY
            #define COLOR_RED \
                FOREGROUND_INTENSITY | FOREGROUND_RED
            #define COLOR_GREEN \
                FOREGROUND_INTENSITY | FOREGROUND_GREEN
            #define COLOR_BLUE \
                FOREGROUND_INTENSITY | FOREGROUND_BLUE
            #define COLOR_YELLOW \
                FOREGROUND_INTENSITY | FOREGROUND_RED | FOREGROUND_GREEN
            #define BACKGROUND_MASK \
                BACKGROUND_RED | \
                BACKGROUND_GREEN | \
                BACKGROUND_BLUE | \
                BACKGROUND_INTENSITY

            CONSOLE_SCREEN_BUFFER_INFO csbi;
            GetConsoleScreenBufferInfo(sConsole, &csbi);
            WORD oldAttributes = csbi.wAttributes;
            WORD bgAttrs = csbi.wAttributes & BACKGROUND_MASK;
            WORD newAttrs = bgAttrs;

            switch (level)
            {
            case LEVEL_DEBUG:
                newAttrs |= COLOR_GREY;
                break;
            case LEVEL_INFO:
                newAttrs |= COLOR_GREEN;
                break;
            case LEVEL_WARNING:
                newAttrs |= COLOR_YELLOW;
                break;
            case LEVEL_ERROR:
                newAttrs |= COLOR_RED;
                break;
            }

            SetConsoleTextAttribute(sConsole, newAttrs);
        #else
            #define COLOR_GREY      "\x1b[30;1m"
            #define COLOR_RED       "\x1b[31m"
            #define COLOR_GREEN     "\x1b[32m"
            #define COLOR_BLUE      "\x1b[34m"
            #define COLOR_YELLOW    "\x1b[33m"
            #define COLOR_RESET     "\x1b[0m"

            switch (level)
            {
            case LEVEL_DEBUG:
                fprintf(stderr, COLOR_GREY);
                break;
            case LEVEL_INFO:
                fprintf(stderr, COLOR_GREEN);
                break;
            case LEVEL_WARNING:
                fprintf(stderr, COLOR_YELLOW);
                break;
            case LEVEL_ERROR:
                fprintf(stderr, COLOR_RED);
                break;
            }
        #endif

        const char * levelStr = 0;
        switch (level)
        {
        case LEVEL_DEBUG:
            levelStr = "DEBUG";
            break;
        case LEVEL_INFO:
            levelStr = "INFO ";
            break;
        case LEVEL_WARNING:
            levelStr = "WARN ";
            break;
        case LEVEL_ERROR:
            levelStr = "ERROR";
            break;
        }

        va_list args;
        va_start( args, format );
        fprintf(stderr, "%s : %s : ", levelStr, tag);
        vfprintf(stderr, format, args);
        va_end( args );

        #if _WIN32
            SetConsoleTextAttribute(sConsole, oldAttributes);
            fprintf(stderr, "\n");
        #else
            fprintf(stderr, COLOR_RESET); // Must precede line feed
            fprintf(stderr, "\n");
        #endif
    #endif
    }

} // namespace Trace
