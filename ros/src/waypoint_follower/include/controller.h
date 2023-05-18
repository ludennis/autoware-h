#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <iostream>
#include <vector>

class Controller
{
public:
    Controller(const std::string & paramFileDir, const float stepSize);
    float OneStep(const float error, const float speed);
    void ResetState();

protected:
    void UpdateCoefficient(const float speed);

private:
    std::string mParamFileDir;
    float mStepSize;
    std::vector<std::vector<float>> mParamA;
    std::vector<std::vector<float>> mParamB;
    std::vector<float> mState;
    std::vector<float> mCoefficientA;
    std::vector<float> mCoefficientB;
};

#endif // __CONTROLLER_H__
