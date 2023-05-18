#ifndef __EPS_DIAGNOSIS_H__
#define __EPS_DIAGNOSIS_H__

#include <iostream>
#include <vector>

enum class EPSDiagnosisState
{
    PASS,
    FAIL
};

class EPSDiagnosis
{
public:
    EPSDiagnosis(
        const float diagnosisDuration,
        const float samplingTime,
        const float spinRateThreshold);

    EPSDiagnosisState Diagnosis(const float steeringAngle);

protected:
    float GetMaxDifferentiateValue();

private:
    float mSamplingTime;
    float mSpinRateThreshold;
    int mResetCount;
    std::vector<float> mSteeringAngleList;
};

#endif // __EPS_DIAGNOSIS_H__
