#include <behavior/Avoidance/hsr_slow_down_action.h>
#include <string>
#include <mutex>

static const float HALF_CAR_WIDTH = 2.4f * 0.5f;

using hash_t = size_t;
constexpr hash_t prime = 0x100000001B3ull;
constexpr hash_t basis = 0xCBF29CE484222325ull;

static hash_t hash_run_time(const char* str) {
    hash_t ret = basis;

    while (*str) {
        ret ^= *str;
        ret *= prime;
        str++;
    }

    return ret;
}

constexpr hash_t hash_compile_time(const char* str, hash_t last_value = basis) {
    return *str ? hash_compile_time(str + 1, (*str ^ last_value) * prime) : last_value;
}

constexpr hash_t operator "" _hash(const char* p, size_t) {
    return hash_compile_time(p);
}

namespace Behavior
{
    static inline float GetAcceleration(
        const float targetSpeed, const float currentSpeed,
        const float targetDistance)
    {
        if (targetSpeed > currentSpeed)
            return 0.0f;

        return 0.5f / targetDistance * (
            targetSpeed * targetSpeed - currentSpeed * currentSpeed);
    }

    static float GetTargetSpeed(const float lateralDistance)
    {
        if (lateralDistance < 0.8f)
            return 5.0f * lateralDistance;
        else
            return 15.5f * (lateralDistance - 0.8f) + 2.7776f;
    }

    static visualization_msgs::Marker InitLineMarker(
        const std::vector<double> & rgb)
    {
        visualization_msgs::Marker marker;
        marker.ns = "points_and_lines";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.2;
        marker.color.r = rgb[0];
        marker.color.g = rgb[1];
        marker.color.b = rgb[2];
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.05);
        return marker;
    }

    hsrSlowDownAction::hsrSlowDownAction(
        std::string name,
        const std::shared_ptr<BehaviorOutput> & nodeCommand,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context,
        hsrAvoidanceSequence & nodeContext)
        : ActionNode::ActionNode(name)
        , mNodeCommand(nodeCommand)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
        , mNodeContext(nodeContext)
        , mAccelerationCmdList()
    {

    }

    BT::ReturnStatus hsrSlowDownAction::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);

        mAccelerationCmdList.clear();
        for (const auto & obj : mContext->objListSD)
        {
            std::vector<float> cmdList;
            for (int i = 0; i < obj.s.size(); ++ i)
            {
                if (!obj.s.size())
                    continue;
                const float pointToVehicleDistance =
                    obj.s[i] - mContext->vehPosSD.s;
                if (pointToVehicleDistance > 1.5f &&
                    pointToVehicleDistance < 100.0f &&
                    std::abs(obj.d[i]) < 3.0f)
                {
                    const int index = 10.0f * (
                        obj.s[i] - mContext->wayPointsS.front());
                    if (!std::signbit(index))
                    {
                        const float pointToWaypointDistance = std::abs(
                            mContext->wayPointsD[index] - obj.d[i])
                                - HALF_CAR_WIDTH;
                        const bool hsrNotSlowDown =
                            mContext->lastClosestPathPoint > 8149 &&
                            mContext->lastClosestPathPoint < 8449;
                        if (!std::signbit(pointToWaypointDistance) &&
                            pointToWaypointDistance < 3.0f)
                        {
                            cmdList.push_back(GetAcceleration(
                                GetTargetSpeed(pointToWaypointDistance),
                                -obj.relativeSpeedS,
                                // mContext->vehPosSD.speed,
                                pointToVehicleDistance));
                        }
                    }
                }
            }

            auto minAccel = std::min_element(
                cmdList.begin(), cmdList.end());
            if (minAccel != cmdList.end())
                mAccelerationCmdList[obj.id] = *minAccel;
        }

        if (mAccelerationCmdList.size())
        {
            const auto minAcceleration = *std::min_element(
                mAccelerationCmdList.begin(), mAccelerationCmdList.end(),
                [] (const std::pair<int, float> & lhs, const std::pair<int, float> & rhs)
                {
                    return lhs.second < rhs.second;
                });

            const float minAccelerationPretected =
                std::max(minAcceleration.second, -1.0f);

            if (std::signbit(minAccelerationPretected))
            {
                mContext->OAmSlowDownObjId = minAcceleration.first;
                SpeedCmd cmd;
                cmd.speed = 5.0f / 3.6f;
                cmd.acceleration = minAccelerationPretected;
                mContext->accelerationCmd["OA"] = cmd;
            }
            else
                mContext->OAmSlowDownObjId = -1;
        }

        Visualization();

        return BT::SUCCESS;
    }


    void hsrSlowDownAction::Halt()
    {
        set_status(BT::HALTED);
    }

    void hsrSlowDownAction::Visualization()
    {
        visualization_msgs::MarkerArray markerList;

        int markerId = 150;
        for (const auto & accelCmd : mAccelerationCmdList)
        {
            visualization_msgs::Marker lineMarker = InitLineMarker(
                {1.0, 1.0, 1.0});
            lineMarker.header.frame_id = "map";
            lineMarker.id = markerId ++;

            const auto objXY = mContext->objLiRarXY.find(accelCmd.first);
            if (objXY != mContext->objLiRarXY.end())
            {
                geometry_msgs::Point markerPt;
                for (size_t i = 0; i < objXY->second.x.size(); ++ i)
                {
                    markerPt.x = objXY->second.x[i];
                    markerPt.y = objXY->second.y[i];
                    markerPt.z = objXY->second.z[i];
                    lineMarker.points.push_back(markerPt);
                }
                markerPt.x = objXY->second.x.front();
                markerPt.y = objXY->second.y.front();
                markerPt.z = objXY->second.z.front();
                lineMarker.points.push_back(markerPt);
                markerList.markers.push_back(lineMarker);
            }
        }

        if (hash_run_time(mContext->tagState.c_str()) == "OA"_hash)
        {
            visualization_msgs::Marker lineMarker = InitLineMarker(
                {1.0, 0.0, 0.0});
            lineMarker.header.frame_id = "map";
            lineMarker.id = markerId ++;
            if (mContext->OAmSlowDownObjId != -1)
            {
                const auto objXY = mContext->objLiRarXY.find(mContext->OAmSlowDownObjId);
                if (objXY != mContext->objLiRarXY.end())
                {
                    geometry_msgs::Point markerPt;
                    for (size_t i = 0; i < objXY->second.x.size(); ++ i)
                    {
                        markerPt.x = objXY->second.x[i];
                        markerPt.y = objXY->second.y[i];
                        markerPt.z = objXY->second.z[i] + 0.3;
                        lineMarker.points.push_back(markerPt);
                    }
                    markerPt.x = objXY->second.x.front();
                    markerPt.y = objXY->second.y.front();
                    markerPt.z = objXY->second.z.front() + 0.3;
                    lineMarker.points.push_back(markerPt);
                    markerList.markers.push_back(lineMarker);
                }
            }
        }

        mContext->pubActionMarkers.publish(markerList);
    }
} // namespace Behavior
