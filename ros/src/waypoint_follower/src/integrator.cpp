#include <integrator.h>

static const float INITIAL_VALUE_ZERO = 0.0f;

Integrator::Integrator(const float stepSize)
    : mStepSize(stepSize)
    , mState(INITIAL_VALUE_ZERO)
{
}

float Integrator::OneStep(const float inputSignal)
{
    mState += mStepSize * inputSignal;
    return mState;
}

void Integrator::ResetState()
{
    mState = INITIAL_VALUE_ZERO;
}
