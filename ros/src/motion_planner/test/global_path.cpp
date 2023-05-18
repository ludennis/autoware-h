#include <file_loader.h>
#include <motion_planner/global_path.h>
#include <gtest/gtest.h>

static const float SMALL_VALUE = 1.0e-03f;

namespace testing {

    class GlobalPath: public Test
    {
    protected:
        GlobalPath()
            : mGlobalPath()
        {}

        void PublishGlobalPath(const std::string & pathFile)
        {
            itri_msgs::Path path;
            itri_msgs::WaypointArray pathTmp;
            LoadPathData(pathFile, pathTmp);
            path.waypoints.assign(
                pathTmp.waypoints.begin(), pathTmp.waypoints.end());
            mGlobalPath = std::make_shared<::GlobalPath>(path);
        }

        float CalculatePathResolution()
        {
            return mGlobalPath->mResolution;
        }

        float CalculatePathCurvature(size_t i)
        {
            return mGlobalPath->mCurvature[i];
        }

    protected:
        std::shared_ptr<::GlobalPath> mGlobalPath;
    };

    TEST_F(GlobalPath, GlobalPathProperty)
    {
        PublishGlobalPath("global_path/high_resolution_path.json");
        ASSERT_NEAR(0.1f, CalculatePathResolution(), SMALL_VALUE);

        std::vector<float> answer;
        LoadAnswerFromFile("global_path/answer_high.json", answer);
        for (size_t i = 0; i < answer.size(); i ++)
        {
            ASSERT_NEAR(answer[i], CalculatePathCurvature(i), SMALL_VALUE);
        }

        PublishGlobalPath("global_path/low_resolution_path.json");
        ASSERT_NEAR(1.0f, CalculatePathResolution(), SMALL_VALUE);

        LoadAnswerFromFile("global_path/answer_low.json", answer);
        for (size_t i = 0; i < answer.size(); i ++)
        {
            ASSERT_NEAR(answer[i], CalculatePathCurvature(i), SMALL_VALUE);
        }
    }

} // namespace testing
