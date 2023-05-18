#include <behavior/lane_change/find_current_lane_action.h>
#include <mutex>

static float SEARCH_RANGE{ 10.0f };

static inline float GetMinAngleDifference(const float angle)
{
    return std::abs(std::atan2(std::sin(angle), std::cos(angle)));
}

namespace Behavior
{
    FindCurrentLaneAction::FindCurrentLaneAction(
        const std::string & name,
        const std::shared_ptr<Context> & context,
        const std::shared_ptr<LaneChangeContext> laneChangeContext)
        : ActionNode::ActionNode(name)
        , mContext(context)
        , mLaneChangeContext(laneChangeContext)
    {}

    BT::ReturnStatus FindCurrentLaneAction::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);

        if (mContext->globalPathKdtree &&
            mContext->globalPathKdtree->getInputCloud() &&
            !!mContext->globalPathKdtree->getInputCloud()->points.size())
        {
            std::vector<int> idxSearch(1);
            std::vector<float> unused(1);
            mContext->globalPathKdtree->nearestKSearch(
                pcl::PointXYZ(mContext->vehPosXY.x, mContext->vehPosXY.y, 0.0),
                1, idxSearch, unused);
            mLaneChangeContext->currentWaypoint =
                mContext->globalPath[idxSearch.front()];
        }
        else if (GlobalSearching() == BT::FAILURE)
            return BT::FAILURE;

        if (is_halted())
            return BT::HALTED;
        return BT::SUCCESS;
    }

    route_mission_handler::Waypoint FindCurrentLaneAction::GetWaypoint(
        const int propertyId)
    {
        const auto property = mLaneChangeContext->naviRoadProperty[propertyId];
        return mContext->laneMap[property.laneId][property.pointId];
    }

    BT::ReturnStatus FindCurrentLaneAction::GlobalSearching()
    {
        std::vector<int> idxSearch;
        std::vector<float> distance;

        if (mLaneChangeContext->naviRoadTree &&
            mLaneChangeContext->naviRoadTree->getInputCloud() &&
            mLaneChangeContext->naviRoadTree->radiusSearch(
                pcl::PointXYZ(mContext->vehPosXY.x, mContext->vehPosXY.y, 0.0),
                SEARCH_RANGE, idxSearch, distance) > 0)
        {
            std::vector<float> normDistance;
            for (size_t i{ 0 }; i < idxSearch.size(); ++ i)
            {
                const auto waypoint = GetWaypoint(idxSearch[i]);
                normDistance.push_back(std::hypot(distance[i] / SEARCH_RANGE,
                    GetMinAngleDifference(
                        waypoint.heading - mContext->vehPosXY.heading) / M_PI));
            }

            const int minIdx = std::distance(normDistance.begin(),
                std::min_element(normDistance.begin(), normDistance.end()));

            mLaneChangeContext->currentWaypoint = GetWaypoint(
                idxSearch[minIdx]);

            return BT::SUCCESS;
        }

        return BT::FAILURE;
    }

    void FindCurrentLaneAction::Halt()
    {
        set_status(BT::HALTED);
    }
} // namespace Behavior
