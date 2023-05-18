#ifndef __LC_LANE_CHANGE_CONTEXT_H__
#define __LC_LANE_CHANGE_CONTEXT_H__

#include <behavior/lane_change/lane_change_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <route_mission_handler/Path.h>
#include <vector>

namespace Behavior
{
    struct LaneChangeContext
    {
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr naviRoadTree;
        std::vector<WaypointProperty> naviRoadProperty;
        route_mission_handler::Waypoint currentWaypoint;
        int targetLaneId;
    };
} // namespace Behavior

#endif // __LC_LANE_CHANGE_CONTEXT_H__
