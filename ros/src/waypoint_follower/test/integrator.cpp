#include <cmath>
#include <gtest/gtest.h>
#include <integrator.h>

static const float SMALL_VALUE = 0.01f;

TEST(Integrator, ManySteps)
{
    Integrator integrator(0.01f);

    for (size_t i = 0; i < 100; i ++)
    {
        const float filterOut = integrator.OneStep(1.0f);
        const float value = 0.01f * static_cast<float>(i + 1);
        ASSERT_NEAR(filterOut, value, SMALL_VALUE);
    }

    integrator.ResetState();
    for (size_t i = 0; i < 100; i ++)
    {
        const float filterOut =
            integrator.OneStep(std::cos(0.01f * static_cast<float>(i)));
        const float value = std::sin(0.01f * static_cast<float>(i + 1));
        ASSERT_NEAR(filterOut, value, SMALL_VALUE);
    }
}
