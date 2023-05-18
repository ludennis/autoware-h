#include <signal_filter.h>

static float const FILTER_CONSTANT = 1.0f;
static const float INITIAL_VALUE_ZERO = 0.0f;

SignalFilter::SignalFilter(const float filterFactor)
    : mSignalFiltered(INITIAL_VALUE_ZERO)
    , mCovariance(FILTER_CONSTANT)
    , mFilterFactor(filterFactor)
{}

float SignalFilter::OneStep(const float signalToFilter)
{
    mCovariance += FILTER_CONSTANT;
    const float filterGain = mCovariance / (mCovariance + mFilterFactor);
    mSignalFiltered += filterGain * (signalToFilter - mSignalFiltered);
    mCovariance = (FILTER_CONSTANT - filterGain) * mCovariance;

    return mSignalFiltered;
}

void SignalFilter::Reset()
{
    mSignalFiltered = INITIAL_VALUE_ZERO;
}
