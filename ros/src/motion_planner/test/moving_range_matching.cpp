#include <gtest/gtest.h>
#include <motion_planner/moving_range_matching.h>

TEST(MovingRangeMatching, RunOnce)
{
    MovingRangeMatching movingRangeMatching;
    itri_msgs::CarState carState;
    itri_msgs::WaypointArray globalPath;

    globalPath.waypoints.resize(50);
    for (size_t i = 0; i < globalPath.waypoints.size(); i++)
    {
        globalPath.waypoints[i].pose.pose.position.x = static_cast<double>(i);
    }

    carState.pose.pose.position.x = 3.1;
    carState.pose.pose.position.y = 1.5;
    carState.pose.pose.orientation.z = 0.0;
    int indexMatch = movingRangeMatching.RunOnce(1.0f, globalPath, carState);
    ASSERT_EQ(indexMatch, 3);

    carState.pose.pose.position.x = 30.4;
    indexMatch = movingRangeMatching.RunOnce(1.0f, globalPath, carState);
    ASSERT_EQ(indexMatch, -1);
    indexMatch = movingRangeMatching.RunOnce(1.0f, globalPath, carState);
    ASSERT_EQ(indexMatch, 30);

    carState.pose.pose.orientation.z = 1.5 * M_PI_2;
    indexMatch = movingRangeMatching.RunOnce(1.0f, globalPath, carState);
    ASSERT_EQ(indexMatch, -1);
    carState.pose.pose.orientation.z = 0.5 * M_PI_2;
    carState.pose.pose.position.x = 45.8;
    indexMatch = movingRangeMatching.RunOnce(1.0f, globalPath, carState);
    ASSERT_EQ(indexMatch, 46);

    carState.pose.pose.position.x = 46.8;
    indexMatch = movingRangeMatching.RunOnce(1.0f, globalPath, carState);
    ASSERT_EQ(indexMatch, 47);
}
