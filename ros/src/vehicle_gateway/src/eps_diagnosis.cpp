#include <algorithm>
#include <eps_diagnosis.h>

EPSDiagnosis::EPSDiagnosis(
    const float diagnosisDuration,
    const float samplingTime,
    const float spinRateThreshold)
    : mSamplingTime(samplingTime)
    , mSpinRateThreshold(spinRateThreshold)
    , mResetCount(0)
    , mSteeringAngleList(
        static_cast<int>(diagnosisDuration / samplingTime), 0.0f)
{}

EPSDiagnosisState EPSDiagnosis::Diagnosis(const float steeringAngle)
{
    std::rotate(mSteeringAngleList.begin(),
        mSteeringAngleList.begin() + 1, mSteeringAngleList.end());
    mSteeringAngleList.back() = steeringAngle;

    if (GetMaxDifferentiateValue() > mSpinRateThreshold)
        return EPSDiagnosisState::FAIL;

    return EPSDiagnosisState::PASS;
}

float EPSDiagnosis::GetMaxDifferentiateValue()
{
    std::vector<float> angleDifference(mSteeringAngleList.size());
    std::adjacent_difference(mSteeringAngleList.begin(),
        mSteeringAngleList.end(), angleDifference.begin());

    std::adjacent_difference(angleDifference.begin(),
        angleDifference.end(), angleDifference.begin());

    for (auto & diffAngle : angleDifference)
        diffAngle = std::abs(diffAngle) / mSamplingTime / mSamplingTime;

    return *std::max_element(
        angleDifference.begin() + 2, angleDifference.end());
}
