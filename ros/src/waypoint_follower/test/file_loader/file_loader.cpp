#include <file_loader.h>
#include <fstream>
#include <gtest/gtest.h>
#include <jsoncpp/json/json.h>

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

void LoadPathData(
    const std::string & fileName, std::vector<float> & pathX,
    std::vector<float> & pathY)
{
    std::vector<std::vector<float>> data2D;
    LoadData2D(fileName, data2D);

    for (size_t i = 0; i < data2D.size(); i ++)
    {
        pathX.push_back(data2D[i][0]);
        pathY.push_back(data2D[i][1]);
    }
}
