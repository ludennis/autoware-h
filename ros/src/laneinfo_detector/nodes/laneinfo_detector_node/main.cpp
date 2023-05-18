#include "node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laneinfo_detector");
    LaneInfoDetector::Node laneinfo;
    laneinfo.MainLoop();
    return 0;
}
