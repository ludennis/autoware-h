#include <behavior/lane_change/find_current_lane_condition.h>
#include <mutex>

namespace Behavior
{
    FindCurrentLaneCondition::FindCurrentLaneCondition(
        const std::string & name,
        const std::shared_ptr<Context> & context,
        const std::shared_ptr<LaneChangeContext> laneChangeContext)
        : ConditionNode::ConditionNode(name)
        , mContext(context)
        , mLaneChangeContext(laneChangeContext)
    {}

    BT::ReturnStatus FindCurrentLaneCondition::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);

        if (!!mContext->laneMap.size())
        {
            if (!!mLaneChangeContext->naviRoadTree &&
                !!mLaneChangeContext->naviRoadTree->getInputCloud() &&
                !!mLaneChangeContext->naviRoadTree->getInputCloud()->points.size())
                return BT::SUCCESS;

            mLaneChangeContext->naviRoadProperty.clear();

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
            cloud->height = 1;
            cloud->points.clear();
            for (const auto & lane : mContext->laneMap)
                for (const auto & waypoint : lane.second)
                {
                    cloud->points.push_back(pcl::PointXYZ(
                        waypoint.second.point.x, waypoint.second.point.y, 0.0));
                    mLaneChangeContext->naviRoadProperty.push_back(
                        WaypointProperty(waypoint.second.point_id,
                            waypoint.second.lane_id, waypoint.second.nroad_id));
                }
            cloud->width = cloud->points.size();

            if (!mLaneChangeContext->naviRoadTree)
                mLaneChangeContext->naviRoadTree =
                    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(
                        new pcl::KdTreeFLANN<pcl::PointXYZ>);

            mLaneChangeContext->naviRoadTree->setInputCloud(cloud);

            return BT::SUCCESS;
        }

        return BT::FAILURE;
    }
} // namespace Behavior
