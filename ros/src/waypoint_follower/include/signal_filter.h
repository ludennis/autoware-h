#ifndef __SIGNAL_FILTER_H__
#define __SIGNAL_FILTER_H__

class SignalFilter
{
public:
    SignalFilter(const float filterFactor);
    float OneStep(const float signalToFilter);
    void Reset();

private:
    float mSignalFiltered;
    float mCovariance;
    float mFilterFactor;
};

#endif // __SIGNAL_FILTER_H__
