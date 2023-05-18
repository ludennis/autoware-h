#include <gtest/gtest.h>
#include <ros/init.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_follower");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
