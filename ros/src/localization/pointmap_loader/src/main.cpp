#include <pointmap_loader.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointmap_loader");
    PointMapLoader lidarMap;

    ros::spin();
    return 0;
}