#ifndef __FILE_LOADER_H__
#define __FILE_LOADER_H__

#include <itri_msgs/DetectedObjectArray.h>
#include <itri_msgs/WaypointArray.h>
#include <string>
#include <vector>

void LoadAnswerFromFile(
    const std::string & fileName, std::vector<float> & answer);

void LoadData2D(
    const std::string & fileName, std::vector<std::vector<float>> & data2D);

void LoadData3D(
    const std::string & fileName,
    std::vector<std::vector<std::vector<float>>> & data3D);

void LoadPathData(
    const std::string & fileName, itri_msgs::WaypointArray & path);

void LoadObjectData(
    const std::string & fileName, itri_msgs::DetectedObjectArray & objectList);

#endif // __FILE_LOADER_H__
