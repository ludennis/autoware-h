#ifndef UTILS_H_
#define UTILS_H_
#include <geometry_msgs/Pose.h>
#include <itri_msgs/WaypointArray.h>

int PathMatching(
    const itri_msgs::WaypointArray & globalPath,
    const geometry_msgs::Pose & point);

#endif /* LANEINFODETECTORCORE_H_ */
