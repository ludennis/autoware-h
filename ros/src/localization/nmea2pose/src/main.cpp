#include <ros/ros.h>
#include <nmea2pose.h>

#define OVERRIDE_NODE_VERBOSITY_LEVEL 1

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmea2pose");
    #if OVERRIDE_NODE_VERBOSITY_LEVEL
        ros::console::set_logger_level(
            ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
    #endif
    Nmea2Pose nmeaToPose;

    ros::spin();

    return 0;
}
