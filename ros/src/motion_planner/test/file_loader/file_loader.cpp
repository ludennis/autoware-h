#include <file_loader.h>
#include <fstream>
#include <gtest/gtest.h>
#include <jsoncpp/json/json.h>
#include <cstdlib>
#include <iomanip>
static Json::Reader JSON_READER;

void LoadAnswerFromFile(
    const std::string & fileName, std::vector<float> & answer)
{
    answer.clear();
    const std::string & fullFileName = TEST_DATA_DIR + fileName;
    std::ifstream file(fullFileName);
    ASSERT_TRUE(file.is_open());
    Json::Value value;
    JSON_READER.parse(file, value);
    for (const auto & answerValue : value["data"])
    {
        ASSERT_TRUE(answerValue.isDouble());
        answer.push_back(answerValue.asFloat());
    }
}

void LoadData2D(
    const std::string & fileName, std::vector<std::vector<float>> & data2D)
{
    data2D.clear();
    std::string fullFileName = TEST_DATA_DIR + fileName;
    std::ifstream file(fullFileName);
    ASSERT_TRUE(file.is_open());
    Json::Value value;
    JSON_READER.parse(file, value);
    for (const auto & dataArray : value["data"])
    {
        std::vector<float> data1D;
        for (const auto & dataValue : dataArray)
        {
            ASSERT_TRUE(dataValue.isDouble());
            data1D.push_back(dataValue.asFloat());
        }
        data2D.push_back(data1D);
    }
}

void LoadData3D(
    const std::string & fileName,
    std::vector<std::vector<std::vector<float>>> & data3D)
{
    data3D.clear();
    std::string fullFileName = TEST_DATA_DIR + fileName;
    std::ifstream file(fullFileName);
    ASSERT_TRUE(file.is_open());
    Json::Value value;
    JSON_READER.parse(file, value);
    for (const auto & dataArray2D : value["data"])
    {
        std::vector<std::vector<float>> data2D;
        for (const auto & dataArray : dataArray2D)
        {
            std::vector<float> data1D;
            for (const auto & dataValue : dataArray)
            {
                ASSERT_TRUE(dataValue.isDouble());
                data1D.push_back(dataValue.asFloat());
            }
            data2D.push_back(data1D);
        }
        data3D.push_back(data2D);
    }
}

void LoadPathData(
    const std::string & fileName, itri_msgs::WaypointArray & path)
{
    std::vector<std::vector<float>> data2D;
    LoadData2D(fileName, data2D);

    path.waypoints.resize(data2D.size());

    for (size_t i = 0; i < data2D.size(); i ++)
    {
        ASSERT_TRUE(data2D[i].size() == 3);
        path.waypoints[i].pose.pose.position.x = data2D[i][0];
        path.waypoints[i].pose.pose.position.y = data2D[i][1];
        path.waypoints[i].pose.pose.orientation.z = data2D[i][2];
    }
}

void LoadObjectData(
    const std::string & fileName, itri_msgs::DetectedObjectArray & objectList)
{
    std::vector<std::vector<std::vector<float>>> data3D;
    LoadData3D(fileName, data3D);

    objectList.objects.resize(data3D.size());
    for (size_t i = 0; i < data3D.size(); i ++)
    {
        objectList.objects[i].id = i;
        objectList.objects[i].convex_hull.polygon.points.resize(
            data3D[i].size());
        for (size_t j = 0; j < data3D[i].size(); j++)
        {
            ASSERT_TRUE(data3D[i][j].size() == 5);
            objectList.objects[i].convex_hull.polygon.points[j].x =
                data3D[i][j][0];
            objectList.objects[i].convex_hull.polygon.points[j].y =
                data3D[i][j][1];
            objectList.objects[i].convex_hull.polygon.points[j].z =
                data3D[i][j][2];
            objectList.objects[i].convex_hull.polygon.points[j].s =
                data3D[i][j][3];
            objectList.objects[i].convex_hull.polygon.points[j].d =
                data3D[i][j][4];
        }

    }
}
