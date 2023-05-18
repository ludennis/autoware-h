#ifndef __INTEGRATOR_H__
#define __INTEGRATOR_H__

class Integrator
{
public:
    Integrator(const float stepSize);
    float OneStep(const float inputSignal);
    void ResetState();

private:
    float mStepSize;
    float mState;
};

#endif // __INTEGRATOR_H__
