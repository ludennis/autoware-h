#include  <algorithm>
#include <behavior/get_curvature.h>
#include <cmath>
#include <cstddef>
#include <numeric>

static inline float Gauss(const float sigma, const float x)
{
    float expVal = -1.0f * (std::pow(x, 2) / std::pow(2.0f * sigma, 2));
    float divider = std::sqrt(2.0f * M_PI * std::pow(sigma, 2));
    return (1.0f / divider) * std::exp(expVal);
}

inline std::vector<float> GaussKernel(int samples, const float sigma)
{
    std::vector<float> v;

    bool floatCenter = false;
    if (samples % 2 == 0)
    {
        floatCenter = true;
        samples --;
    }
    int steps = (samples - 1) / 2;
    float stepSize = (3.0f * sigma) / steps;

    for (int i = steps; i >= 1; -- i)
        v.push_back(Gauss(sigma, -i * stepSize));

    v.push_back(Gauss(sigma, 0));
    if (floatCenter)
        v.push_back(Gauss(sigma, 0));

    for (int i = 1; i <= steps; ++ i)
        v.push_back(Gauss(sigma, i * stepSize));

    return v;
}

inline std::vector<float> GaussSmoothen(
    const std::vector<float> & values, const float sigma, const int samples)
{
    std::vector<float> out;
    auto kernel = GaussKernel(samples, sigma);
    int sampleSide = samples / 2;
    int valueIdx = samples / 2 + 1;
    for (size_t i = 0; i < values.size(); ++ i)
    {
        float sample = 0.0f;
        int sampleCtr = 1;
        for (auto j = i - sampleSide; j <= i + sampleSide; ++ j)
        {
            if (j > 0u && j < values.size())
            {
                sample += kernel[sampleSide + j - i] * values[j];
                sampleCtr ++;
            }
        }
        out.push_back(sigma * float(samples) * sample / float(sampleCtr));
    }
    return out;
}

static void Curvature(
    const std::vector<float> & x,
    const std::vector<float> & y,
    std::vector<float> & curvature)
{
    std::vector<float> diffX(x);
    std::vector<float> diffY(y);
    std::vector<float> diff2X(x);
    std::vector<float> diff2Y(x);

    std::adjacent_difference(x.begin(), x.end(), diffX.begin());
    std::adjacent_difference(y.begin(), y.end(), diffY.begin());
    const auto keepLastDiffX = diffX.back();
    const auto keepLastDiffY = diffY.back();
    std::rotate(diffX.begin(), diffX.begin() + 1, diffX.end());
    std::rotate(diffY.begin(), diffY.begin() + 1, diffY.end());
    diffX.back() = keepLastDiffX;
    diffY.back() = keepLastDiffY;

    std::adjacent_difference(diffX.begin(), diffX.end(), diff2X.begin());
    std::adjacent_difference(diffY.begin(), diffY.end(), diff2Y.begin());
    const auto keepLastDiff2X = diff2X.back();
    const auto keepLastDiff2Y = diff2Y.back();
    std::rotate(diff2X.begin(), diff2X.begin() + 1, diff2X.end());
    std::rotate(diff2Y.begin(), diff2Y.begin() + 1, diff2Y.end());
    diff2X.back() = keepLastDiff2X;
    diff2Y.back() = keepLastDiff2Y;

    curvature.clear();
    for (size_t i{ 0 }; i < x.size(); ++ i)
        curvature.push_back((diffX[i] * diff2Y[i] - diffY[i] * diff2X[i]) /
            std::pow(std::hypot(diffX[i], diffY[i]), 3));
}

namespace Behavior
{
    void GetCurvature(
        const std::vector<float> & x,
        const std::vector<float> & y,
        std::vector<float> & curvature)
    {
        Curvature(x, y, curvature);
        curvature = GaussSmoothen(curvature, 2.0f, 8);
    }
}/* namespace Behavior */
