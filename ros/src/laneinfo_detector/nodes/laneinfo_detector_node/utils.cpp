#include <utils.h>

int PathMatching(
    const itri_msgs::WaypointArray & globalPath,
    const geometry_msgs::Pose & point)
{
    std::vector<float> distance(globalPath.waypoints.size(), 0.0f);
    for (size_t i = 0; i < distance.size(); i++)
    {
        const float diffX = globalPath.waypoints[i].pose.pose.position.x -
            point.position.x;
        const float diffY = globalPath.waypoints[i].pose.pose.position.y -
            point.position.y;
        distance[i] = std::hypot(diffX, diffY);
    }

    return std::distance(distance.begin(),
        std::min_element(distance.begin(), distance.end()));
}
