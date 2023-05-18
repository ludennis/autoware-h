#include <debug_to_file/debug_to_file.h>
#include <fstream>
#include <ros/time.h>

DebugToFile::DebugToFile(const std::string & fileName)
    : mDebugValue(Json::arrayValue)
    , mFileName(DEBUG_DATA_DIR)
    , mThisTimeStamp()
    , mTurnLoggerOn(true)
{
    mFileName += fileName;
}

DebugToFile::~DebugToFile()
{
    if (mTurnLoggerOn)
    {
        std::ofstream file;
        file.open(mFileName);
        Json::Value jsonRootValue;
        jsonRootValue["data"] = mDebugValue;
        Json::StyledWriter styledWriter;
        file << styledWriter.write(jsonRootValue);
        file.close();
    }
}
