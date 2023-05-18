#ifndef __DEBUG_TO_FILE_H__
#define __DEBUG_TO_FILE_H__

#include <iostream>
#include <jsoncpp/json/json.h>
#include <string>

class DebugToFile
{
public:
    DebugToFile(const std::string & fileName);
    ~DebugToFile();

    inline void StartAddDebug(const double timeNow)
    {
        if (mTurnLoggerOn)
            mThisTimeStamp["time"] = timeNow;
    }

    template <typename T>
    inline void AddDebug(const std::string & message, const T value)
    {
        std::cout << message << ": " << value << '\n';
        if (mTurnLoggerOn)
            mThisTimeStamp[message] = std::to_string(value);
    }

    inline void StopAddDebug()
    {
        if (mTurnLoggerOn)
        {
            mDebugValue.append(mThisTimeStamp);
            mThisTimeStamp.clear();
        }
    }

    inline void SetLogOptionOff()
    {
        mTurnLoggerOn = false;
    }

private:
    Json::Value mDebugValue;
    std::string mFileName;
    Json::Value mThisTimeStamp;
    bool mTurnLoggerOn;
};

#endif // __DEBUG_TO_FILE_H__
