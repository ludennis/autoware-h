#ifndef __FILE_LOADER_H__
#define __FILE_LOADER_H__

#include <string>
#include <vector>

void LoadAnswerFromFile(
    const std::string & fileName, std::vector<float> & answer);

void LoadData2D(
    const std::string & fileName, std::vector<std::vector<float>> & data2D);

void LoadPathData(
    const std::string & fileName, std::vector<float> & pathX,
    std::vector<float> & pathY);

#endif // __FILE_LOADER_H__
