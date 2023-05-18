#ifndef __GLOBAL_PATH_H__
#define __GLOBAL_PATH_H__

#include <itri_msgs/Path.h>
#include <itri_msgs/WaypointArray.h>

class GlobalPath
{
public:
    GlobalPath(const itri_msgs::Path &);

protected:
    void CalculatePathResolution();
    void CalculatePathCurvature();

public:
    itri_msgs::WaypointArray mPath;
    std::vector<float> mCurvature;
    float mResolution;
};

#endif // __GLOBAL_PATH_H__
