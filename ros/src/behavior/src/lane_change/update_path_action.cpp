#include <algorithm>
#include <behavior/lane_change/update_path_action.h>
#include <mutex>
#include <vector>

const static float RANGE_THRESHOLD = 0.5f;

namespace Behavior
{
    UpdatePathAction::UpdatePathAction(
        const std::string & name,
        const std::shared_ptr<Context> & context,
        const std::shared_ptr<LaneChangeContext> laneChangeContext)
        : ActionNode::ActionNode(name)
        , mContext(context)
        , mLaneChangeContext(laneChangeContext)
        , mSelectedLaneId()
        , mTravelMap()
    {}

    BT::ReturnStatus UpdatePathAction::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);

        if (!mContext->laneInfoMap.size())
            return BT::FAILURE;

        mTravelMap.clear();
        for (const auto & lane : mContext->laneInfoMap)
            mTravelMap[lane.first] = false;

        mSelectedLaneId.clear();
        ConnectLane(mLaneChangeContext->targetLaneId);
        if (ConstructGlobalPath() == BT::FAILURE)
            return BT::FAILURE;

        if (is_halted())
            return BT::HALTED;
        return BT::SUCCESS;
    }

    void UpdatePathAction::ConnectLane(const int laneId)
    {
        if (!mTravelMap[laneId])
        {
            mSelectedLaneId.push_back(laneId);
            mTravelMap[laneId] = true;

            if (!!mContext->laneInfoMap[laneId].next_lanes.size())
            {
                int selectedId =
                    mContext->laneInfoMap[laneId].next_lanes.front();
                for (const auto id : mContext->laneInfoMap[laneId].next_lanes)
                {
                    if (!!mContext->laneMap[id].size())
                    {
                        selectedId = id;
                        break;
                    }
                }
                ConnectLane(selectedId);
            }
        }
    }

    BT::ReturnStatus UpdatePathAction::ConstructGlobalPath()
    {
        mContext->globalPath.clear();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        cloud->height = 1;
        cloud->points.clear();

        for (const auto id : mSelectedLaneId)
            for (const auto & waypoint : mContext->laneMap[id])
            {
                bool rangeCheck = true;
                if (mContext->globalPath.size() > 0)
                    rangeCheck = std::hypot(
                        mContext->globalPath.back().point.x -
                            waypoint.second.point.x,
                        mContext->globalPath.back().point.y -
                            waypoint.second.point.y) > RANGE_THRESHOLD;
                if (rangeCheck)
                {
                    mContext->globalPath.push_back(waypoint.second);
                    cloud->points.push_back(pcl::PointXYZ(
                        waypoint.second.point.x, waypoint.second.point.y, 0.0));
                }
            }
        cloud->width = cloud->points.size();

        if (!mContext->globalPathKdtree)
            mContext->globalPathKdtree = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(
                new pcl::KdTreeFLANN<pcl::PointXYZ>);

        if (!cloud->points.size())
            return BT::FAILURE;

        mContext->globalPathKdtree->setInputCloud(cloud);
        return BT::SUCCESS;
    }

    void UpdatePathAction::Halt()
    {
        set_status(BT::HALTED);
    }
} // namespace Behavior
