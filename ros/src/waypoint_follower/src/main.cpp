#include <ros/init.h>
#include <waypoint_follower_node.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_follower");
    WaypointFollowerNode waypointFollowerNode;

    waypointFollowerNode.MainLoop();

    return 0;
}
