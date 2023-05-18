#include <cmath>
#include <eps_controller.h>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <trace/utils.h>

#define TRACE_TAG "EPSController"

static const float INITIAL_VALUE_ZERO = 0.0f;
static const float ODE_PARAM1 = 0.5f;
static const float ODE_PARAM2 = 0.75f;
static const float ODE_PARAM3[3] = {2.0f / 9.0f, 1.0f / 3.0f, 4.0f / 9.0f};
static const float ODE_PARAM4[4] = {7.0f / 24.0f, 0.25f, 1.0f / 3.0f, 0.125f};

static inline void StateFunction(
    const float input, const std::vector<float> & denominator,
    const std::vector<float> & stateIn, std::vector<float> & stateOut)
{
    if (stateOut.size())
    {
        stateOut.front() = input;
        for (size_t i = 0; i < stateOut.size(); i ++)
        {
            stateOut.front() += denominator[i] * stateIn[i];
            if ((i + 1) < stateIn.size())
                stateOut[i + 1] = stateIn[i];
        }
    }
}

static inline void StateUpdate(
    const float factor, const float stepSize,
    const std::vector<float> & stateIn, std::vector<float> & stateOut)
{
    if (stateOut.size())
        for (size_t i = 0; i < stateOut.size(); i ++)
            stateOut[i] += factor * stepSize * stateIn[i];
}

static void LoadParamFromFile(
    const std::string & fileName,
    const std::string & parameterName,
    std::vector<float> & numerator,
    std::vector<float> & denominator)
{
    numerator.clear();
    denominator.clear();
    std::ifstream file(fileName.c_str());
    TRACE_ASSERT_THROW(file.is_open());
    Json::Value value;
    Json::Reader jonsReader;
    jonsReader.parse(file, value);

    std::vector<float> paramDen;
    std::vector<float> paramNum;
    for (const auto & answerValue : value[parameterName][0])
    {
        TRACE_ASSERT_THROW(answerValue.isDouble());
        paramNum.push_back(answerValue.asFloat());
    }
    for (const auto & answerValue : value[parameterName][1])
    {
        TRACE_ASSERT_THROW(answerValue.isDouble());
        paramDen.push_back(answerValue.asFloat());
    }
    TRACE_ASSERT_THROW(paramNum.size() > 0);
    TRACE_ASSERT_THROW(paramDen.size() > 0);
    TRACE_ASSERT_THROW(paramNum.size() == paramDen.size());
    for (size_t i = 1; i < paramDen.size(); ++ i)
        denominator.push_back(-paramDen[i] / paramDen[0]);
    for (size_t i = 1; i < paramNum.size(); ++ i)
    {
        paramNum[i] /= paramDen[0];
        paramNum[i] += paramNum[0] * denominator[i];
        numerator.push_back(paramNum[i]);
    }
    TRACE_ASSERT_THROW(paramDen[0] > 0.0f);
    numerator.push_back(paramNum[0] / paramDen[0]);
}

EPSController::EPSController(
    const std::string & filename, const std::string & parameterName,
    const float stepSize)
    : mStepSize(stepSize)
    , mState()
    , mNumerator()
    , mDenominator()
{
    LoadParamFromFile(filename, parameterName, mNumerator, mDenominator);
    mState.resize(mDenominator.size(), INITIAL_VALUE_ZERO);
}

// Variable naming follows Bogacki–Shampine method (ode23)
float EPSController::OneStep(const float error)
{
    std::vector<float> k1(mState.size(), INITIAL_VALUE_ZERO);
    StateFunction(error, mDenominator, mState, k1);

    std::vector<float> k2(mState.size(), INITIAL_VALUE_ZERO);
    std::vector<float> state1(mState.begin(), mState.end());
    StateUpdate(ODE_PARAM1, mStepSize, k1, state1);
    StateFunction(error, mDenominator, state1, k2);

    std::vector<float> k3(mState.size(), INITIAL_VALUE_ZERO);
    std::vector<float> state2(mState.begin(), mState.end());
    StateUpdate(ODE_PARAM2, mStepSize, k2, state2);
    StateFunction(error, mDenominator, state2, k3);

    std::vector<float> k4(mState.size(), INITIAL_VALUE_ZERO);
    std::vector<float> state3(mState.begin(), mState.end());
    StateUpdate(ODE_PARAM3[0], mStepSize, k1, state3);
    StateUpdate(ODE_PARAM3[1], mStepSize, k2, state3);
    StateUpdate(ODE_PARAM3[2], mStepSize, k3, state3);
    StateFunction(error, mDenominator, state3, k4);

    StateUpdate(ODE_PARAM4[0], mStepSize, k1, mState);
    StateUpdate(ODE_PARAM4[1], mStepSize, k2, mState);
    StateUpdate(ODE_PARAM4[2], mStepSize, k3, mState);
    StateUpdate(ODE_PARAM4[3], mStepSize, k4, mState);

    float controllerCommand = mNumerator.back() * error;
    for (size_t i = 0; i < mState.size(); i ++)
        controllerCommand += mNumerator[i] * mState[i];

    return controllerCommand;
}

void EPSController::ResetState()
{
    std::fill(mState.begin(), mState.end(), INITIAL_VALUE_ZERO);
}
