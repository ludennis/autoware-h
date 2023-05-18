#include <controller.h>
#include <file_loader.h>
#include <gtest/gtest.h>

static const float SMALL_VALUE = 0.001f;

TEST(Controller, Param1)
{
    const std::string paramDir = "param1/";
    Controller controller(TEST_DATA_DIR + paramDir, 0.01f);
    std::vector<float> answer;

    LoadAnswerFromFile("param1/answer1.json", answer);
    for (auto const & value : answer)
    {
        const float controllerOut = controller.OneStep(1.0f, 0.0f);
        ASSERT_NEAR(controllerOut, value, SMALL_VALUE);
    }

    controller.ResetState();
    LoadAnswerFromFile("param1/answer2.json", answer);
    for (auto const & value : answer)
    {
        const float controllerOut = controller.OneStep(1.0f, 1.0f);
        ASSERT_NEAR(controllerOut, value, SMALL_VALUE);
    }
}

TEST(Controller, Param2)
{
    const std::string paramDir = "param2/";
    Controller controller(TEST_DATA_DIR + paramDir, 0.01f);
    std::vector<float> answer;

    LoadAnswerFromFile("param2/answer1.json", answer);
    for (auto const & value : answer)
    {
        const float controllerOut = controller.OneStep(1.0f, 0.0f);
        ASSERT_NEAR(controllerOut, value, SMALL_VALUE);
    }

    controller.ResetState();
    LoadAnswerFromFile("param2/answer2.json", answer);
    for (auto const & value : answer)
    {
        const float controllerOut = controller.OneStep(1.0f, 1.0f);
        ASSERT_NEAR(controllerOut, value, SMALL_VALUE);
    }
}

TEST(Controller, PureGainControl)
{
    const std::string paramDir = "param3/";
    Controller controller(TEST_DATA_DIR + paramDir, 0.01f);

    for (float vlaue = 0.0f; vlaue < 100.0f; vlaue ++)
    {
        const float controllerOut = controller.OneStep(vlaue, 0.0f);
        ASSERT_NEAR(controllerOut, vlaue, SMALL_VALUE);
    }

    controller.ResetState();
    for (float vlaue = 0.0f; vlaue < 100.0f; vlaue ++)
    {
        const float controllerOut = controller.OneStep(vlaue, 1.0f);
        ASSERT_NEAR(controllerOut, 3.0 * vlaue, SMALL_VALUE);
    }

    controller.ResetState();
    for (float vlaue = 0.0f; vlaue < 100.0f; vlaue ++)
    {
        const float controllerOut = controller.OneStep(vlaue, 2.0f);
        ASSERT_NEAR(controllerOut, 7.0 * vlaue, SMALL_VALUE);
    }
}
