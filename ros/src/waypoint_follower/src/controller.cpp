#include <cmath>
#include <controller.h>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <trace/utils.h>

#define TRACE_TAG "Controller"

static const float INITIAL_VALUE_ZERO = 0.0f;
static const float ODE_PARAM1 = 0.5f;
static const float ODE_PARAM2 = 0.75f;
static const float ODE_PARAM3[3] = {2.0f / 9.0f, 1.0f / 3.0f, 4.0f / 9.0f};
static const float ODE_PARAM4[4] = {7.0f / 24.0f, 0.25f, 1.0f / 3.0f, 0.125f};
static const std::string TABLE_A = "tableA.json";
static const std::string TABLE_B = "tableB.json";

static inline float Polynomial(
    const std::vector<float> coefficient, const float x)
{
    float y = INITIAL_VALUE_ZERO;
    for (size_t i = 0; i < coefficient.size(); i ++)
    {
        y += coefficient[i] * std::pow(x, i);
    }

    return y;
}

static inline void StateFunction(
    const float input, const std::vector<float> & coefficientA,
    const std::vector<float> & stateIn, std::vector<float> & stateOut)
{
    stateOut.front() = input;
    for (size_t i = 0; i < stateOut.size(); i ++)
    {
        stateOut.front() += coefficientA[i] * stateIn[i];
        if ((i + 1) < stateIn.size())
            stateOut[i + 1] = stateIn[i];
    }
}

static inline void StateUpdate(
    const float factor, const float stepSize,
    const std::vector<float> & stateIn, std::vector<float> & stateOut)
{
    for (size_t i = 0; i < stateOut.size(); i ++)
    {
        stateOut[i] += factor * stepSize * stateIn[i];
    }
}

static void LoadParamFromFile(
    const std::string & fileName, std::vector<std::vector<float>> & parameter)
{
    parameter.clear();
    std::ifstream file(fileName);
    TRACE_ASSERT_THROW(file.is_open());
    Json::Value value;
    Json::Reader jonsReader;
    jonsReader.parse(file, value);
    for (const auto & answeArray : value["parameters"])
    {
        std::vector<float> parameter1D;
        for (const auto & answerValue : answeArray)
        {
            TRACE_ASSERT_THROW(answerValue.isDouble());
            parameter1D.push_back(answerValue.asFloat());
        }
        parameter.push_back(parameter1D);
    }
}

Controller::Controller(const std::string & paramFileDir, const float stepSize)
    : mParamFileDir(paramFileDir)
    , mStepSize(stepSize)
    , mParamA()
    , mParamB()
    , mState()
    , mCoefficientA()
    , mCoefficientB()
{
    LoadParamFromFile(mParamFileDir + TABLE_A, mParamA);
    LoadParamFromFile(mParamFileDir + TABLE_B, mParamB);
    TRACE_ASSERT_THROW(mParamB.size() == mParamA.size() + 1);

    mState.resize(mParamA.size(), INITIAL_VALUE_ZERO);
    mCoefficientA.resize(mParamA.size(), INITIAL_VALUE_ZERO);
    mCoefficientB.resize(mParamB.size(), INITIAL_VALUE_ZERO);
}

// Variable naming follows Bogacki–Shampine method (ode23)
float Controller::OneStep(const float error, const float speed)
{
    UpdateCoefficient(speed);

    std::vector<float> k1(mState.size(), INITIAL_VALUE_ZERO);
    StateFunction(error, mCoefficientA, mState, k1);

    std::vector<float> k2(mState.size(), INITIAL_VALUE_ZERO);
    std::vector<float> state1(mState.begin(), mState.end());
    StateUpdate(ODE_PARAM1, mStepSize, k1, state1);
    StateFunction(error, mCoefficientA, state1, k2);

    std::vector<float> k3(mState.size(), INITIAL_VALUE_ZERO);
    std::vector<float> state2(mState.begin(), mState.end());
    StateUpdate(ODE_PARAM2, mStepSize, k2, state2);
    StateFunction(error, mCoefficientA, state2, k3);

    std::vector<float> k4(mState.size(), INITIAL_VALUE_ZERO);
    std::vector<float> state3(mState.begin(), mState.end());
    StateUpdate(ODE_PARAM3[0], mStepSize, k1, state3);
    StateUpdate(ODE_PARAM3[1], mStepSize, k2, state3);
    StateUpdate(ODE_PARAM3[2], mStepSize, k3, state3);
    StateFunction(error, mCoefficientA, state3, k4);

    StateUpdate(ODE_PARAM4[0], mStepSize, k1, mState);
    StateUpdate(ODE_PARAM4[1], mStepSize, k2, mState);
    StateUpdate(ODE_PARAM4[2], mStepSize, k3, mState);
    StateUpdate(ODE_PARAM4[3], mStepSize, k4, mState);

    float controllerCommand = mCoefficientB.back() * error;
    for (size_t i = 0; i < mState.size(); i ++)
    {
        controllerCommand += mCoefficientB[i] * mState[i];
    }

    return controllerCommand;
}

void Controller::ResetState()
{
    std::fill(mState.begin(), mState.end(), INITIAL_VALUE_ZERO);
}

void Controller::UpdateCoefficient(const float speed)
{
    for (size_t i = 0; i < mCoefficientA.size(); i ++)
    {
        mCoefficientA[i] = Polynomial(mParamA[i], speed);
    }

    for (size_t i = 0; i < mCoefficientB.size(); i ++)
    {
        mCoefficientB[i] = Polynomial(mParamB[i], speed);
    }
}
