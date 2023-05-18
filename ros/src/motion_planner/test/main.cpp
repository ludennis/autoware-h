#include <gtest/gtest.h>
#include <ros/init.h>

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "motion_planner_tests");
    return RUN_ALL_TESTS();
}
