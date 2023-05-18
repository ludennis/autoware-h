#ifndef __EPS_CONTROLLER_H__
#define __EPS_CONTROLLER_H__

#include <iostream>
#include <string>
#include <vector>

class EPSController
{
public:
    EPSController(
      const std::string & filename, const std::string & parameterName,
      const float stepSize);
    float OneStep(const float error);
    void ResetState();

private:
    float mStepSize;
    std::vector<float> mState;
    std::vector<float> mNumerator;
    std::vector<float> mDenominator;
};

#endif // __EPS_CONTROLLER_H__
