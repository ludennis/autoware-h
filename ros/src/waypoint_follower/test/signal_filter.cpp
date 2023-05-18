#include <file_loader.h>
#include <gtest/gtest.h>
#include <signal_filter.h>

static const float SMALL_VALUE = 0.001f;

TEST(SignalFilter, ManySteps)
{
    SignalFilter filter(100.0f);

    std::vector<float> answer;
    LoadAnswerFromFile("filter/answer.json", answer);
    for (auto const & value : answer)
    {
        const float filterOut = filter.OneStep(1.0f);
        ASSERT_NEAR(filterOut, value, SMALL_VALUE);
    }
}
