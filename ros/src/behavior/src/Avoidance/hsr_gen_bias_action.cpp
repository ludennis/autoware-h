#include <behavior/Avoidance/hsr_gen_bias_action.h>
#include <string>
#include <mutex>
#include <visualization_msgs/Marker.h>

// #define OA_DEBUG
static const float CAR_WIDTH = 2.7f;
const float LIMIT_BIAS_RIGHT = -0.6f;

template <typename T>
static inline T Clamp(const T & value, T bottom, T top)
{
    return std::max(bottom, std::min(top, value));
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

namespace Behavior
{
    genBiasAction::genBiasAction(
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
    {
    }

    BT::ReturnStatus genBiasAction::Tick()
    {
        BT::ReturnStatus btStatus = BT::FAILURE;
        mBehaviorPlanOut.hint = false;
        mContext->LaneChange = false;

        std::lock_guard<std::mutex> guard(mContext->mutex);
        float bias;

        const auto onRoadObstacles = mNodeContext.GetOnRoadObstacles();
        float allowRightBias = 0.0f;
        if (onRoadObstacles.size() > 0)
        {
            const float clearance = 1.3f;
            ObstacleAvoidance OA;
            const auto minRightBoundObstacle = *std::min_element(
                onRoadObstacles.begin(), onRoadObstacles.end(),
                [&] (const ObjectSD & lhs, const ObjectSD & rhs)
                {
                    ObjRectBound lhsBound = OA.FindObjBound(lhs);
                    ObjRectBound rhsBound = OA.FindObjBound(rhs);
                    return lhsBound.rightBound < rhsBound.rightBound;
                });
            const float obstacleRightBound =
                OA.FindObjBound(minRightBoundObstacle).rightBound;
            const float egoLeftBound = mNodeContext.GetBias() +
                CAR_WIDTH / 2.0f;
            allowRightBias = Clamp(obstacleRightBound - egoLeftBound - clearance,
                LIMIT_BIAS_RIGHT, 0.0f);
        }

        mContext->OAGenBias = false;
        if (mNodeContext.GetSideObstacles().size() != 0)
        {
            bias = FindMaxBias();
            if(WithoutBackObstacle(bias))
            {
                if (onRoadObstacles.size() > 0)
                    bias = allowRightBias;

                float outputbias =
                    (bias >= mNodeContext.GetBias())? std::min(bias, mNodeContext.GetBias() + 0.1f):std::max(bias, mNodeContext.GetBias() - 0.05f);
                mNodeContext.SetBias(outputbias);
                mBehaviorPlanOut.hint = true;
                mBehaviorPlanOut.bias = std::max(outputbias, LIMIT_BIAS_RIGHT);
                mBehaviorPlanOut.update_bias = true;
                mContext->OAGenBias = true;

#ifdef OA_DEBUG
                ROS_INFO_STREAM("The Action " << get_name() << " has succeeded");
#endif
                mContext->LaneChange = true;
                btStatus = BT::SUCCESS;
                Visualization();
            }
        }

#ifdef OA_DEBUG
        mContext->debug->AddDebug("sidOBJsize",mNodeContext.GetSideObstacles().size());
#endif

        if (mNodeContext.GetSideObstacles().size() == 0)
        {
            if(WithoutBackObstacle(0.0f))
            {
                float bias = 0.0f;
                if (onRoadObstacles.size() > 0)
                    bias = allowRightBias;

                float outputbias =
                    (bias >= mNodeContext.GetBias())? std::min(bias, mNodeContext.GetBias() + 0.01f):std::max(bias, mNodeContext.GetBias() - 0.04f);
                mNodeContext.SetBias(outputbias);

                if (mNodeContext.GetBias() != 0.0f)
                {
                    mBehaviorPlanOut.hint = true;
                    mContext->OAGenBias = true;
                }
                else
                {
                    mBehaviorPlanOut.hint = false;
                    mContext->OAGenBias = false;
                }

                mBehaviorPlanOut.bias = mNodeContext.GetBias();
                mBehaviorPlanOut.update_bias = true;
                mContext->LaneChange = true;
                btStatus = BT::SUCCESS;
                Visualization();
            }
        }

#ifdef OA_DEBUG
        ROS_INFO_STREAM("The Action " << get_name() << " is failed");
#endif
        std::cout << " GGG mContext->OAGenBias = " << (mContext->OAGenBias ? "True" : "False") <<'\n';

        mContext->pubBehaviorPlan.publish(mBehaviorPlanOut);


        return btStatus;
    }

    void genBiasAction::Halt()
    {
        set_status(BT::HALTED);
    }

    float genBiasAction::FindMaxBias()
    {
        ObstacleAvoidance OA;
        float maxBias;
        float biasByRelativeSpeed =
            0.8f + 0.07f * (mContext->vehPosSD.speed - 2.8f);
        biasByRelativeSpeed = std::max(std::min(biasByRelativeSpeed, 1.3f), 0.8f);

        if (mNodeContext.GetCheckLeftArea())
            maxBias = mNodeContext.GetBias() - 0.5f * CAR_WIDTH - biasByRelativeSpeed;

        else if (mNodeContext.GetCheckRightArea())
            maxBias = mNodeContext.GetBias() + 0.5f * CAR_WIDTH + biasByRelativeSpeed;

        else
             maxBias = mNodeContext.GetBias();

        std::vector<float> leftBoundArray;
        std::vector<int> leftIds;
        for (int i = 0; i < mNodeContext.GetSideObstacles().size(); i++)
        {
            ObjRectBound frontObjectBound =
                OA.FindObjBound(mNodeContext.GetSideObstacles()[i]);

            leftBoundArray.push_back(frontObjectBound.leftBound);
            leftIds.push_back(mNodeContext.GetSideObstacles()[i].id);
        }

        maxBias = *std::max_element(leftBoundArray.begin(), leftBoundArray.end());
        int maxBiasId = std::max_element(leftBoundArray.begin(), leftBoundArray.end()) - leftBoundArray.begin();
        mContext->OAmMaxBiasId = leftIds[maxBiasId];

        maxBias = maxBias + 0.5 * CAR_WIDTH + biasByRelativeSpeed;

        return maxBias;
    }

    bool genBiasAction::WithoutBackObstacle(const float bias)
    {
        float biasByRelativeSpeed =
            0.8f + 0.07f * (mContext->vehPosSD.speed - 2.8f);
        biasByRelativeSpeed = std::max(std::min(biasByRelativeSpeed, 1.3f), 0.8f);

        int backObstacleId = -1;
        bool withoutBackObstacle = true;
        float minBoundary = std::min(bias, mContext->vehPosSD.d - 0.5f * CAR_WIDTH - 0.5f);
        float maxBoundary = std::max(bias, mContext->vehPosSD.d + 0.5f * CAR_WIDTH + 0.5f);

        if (bias <= mContext->vehPosSD.d + 0.5f * CAR_WIDTH + 0.5f &&
            bias >= mContext->vehPosSD.d - 0.5f * CAR_WIDTH - 0.5f)
        {
            return withoutBackObstacle;
        }

        if(bias > mContext->vehPosSD.d + 0.5f * CAR_WIDTH + 0.5f)
        {
            minBoundary =  mContext->vehPosSD.d + 0.5f * CAR_WIDTH;
            maxBoundary = bias;
        }
        if (bias < mContext->vehPosSD.d - 0.5f * CAR_WIDTH - 0.5f)
        {
            minBoundary =  bias;
            maxBoundary = mContext->vehPosSD.d - 0.5f * CAR_WIDTH;
        }

        for (int i = 0; i < mContext->objLiRarSD.size(); i++)
        {
            for (int j = 0; j < mContext->objLiRarSD[i].s.size(); j++)
            {
                if ((mContext->objLiRarSD[i].s[j] - mContext->vehPosSD.s <= 0.0f &&
                    mContext->objLiRarSD[i].s[j] - mContext->vehPosSD.s >= -50.0f) &&
                    mContext->objLiRarSD[i].d[j] <= maxBoundary &&
                    mContext->objLiRarSD[i].d[j] >= minBoundary)
                {
                    if (std::pow(mContext->objLiRarSD[i].relativeSpeedS, 2) / 2.0f >
                    std::abs(mContext->objLiRarSD[i].s[j] - mContext->vehPosSD.s) &&
                    mContext->objLiRarSD[i].relativeSpeedS > 0.0f)
                    {
                        withoutBackObstacle = false;
                        backObstacleId = mContext->objLiRarSD[i].id;
                    }

                    if ((mContext->objLiRarSD[i].s[j] - mContext->vehPosSD.s <= 0.0f &&
                        mContext->objLiRarSD[i].s[j] - mContext->vehPosSD.s >= -15.0f) &&
                        mContext->objLiRarSD[i].d[j] >=  mContext->vehPosSD.d + 0.5f * CAR_WIDTH &&
                        mContext->objLiRarSD[i].d[j] <= mContext->vehPosSD.d + CAR_WIDTH * 0.5f + biasByRelativeSpeed)
                    {
                        withoutBackObstacle = false;
                        backObstacleId = mContext->objLiRarSD[i].id;
                    }
                }
            }
        }

        return withoutBackObstacle;
    }

    bool genBiasAction::IsBiasNeedToChange(const float bias)
    {
        ObstacleAvoidance OA;

        bool biasNeedToChange = false;
        float maxBias;

        if (mNodeContext.GetCheckLeftArea())
        {
            maxBias = mNodeContext.GetBias() - 0.5f * CAR_WIDTH;
        }
        else if (mNodeContext.GetCheckRightArea())
        {
            maxBias = mNodeContext.GetBias() + 0.5f * CAR_WIDTH;
        }
        else
        {
            maxBias = mNodeContext.GetBias();
        }

        bool initialBiasCondition = (mNodeContext.GetBias() == 0);
        bool newBiasLargeEnough = false;

        for (int i = 0; i < mNodeContext.GetSideObstacles().size(); i++)
        {
            ObjRectBound frontObjectBound =
                OA.FindObjBound(mNodeContext.GetSideObstacles()[i]);

            if (mNodeContext.GetCheckLeftArea())
            {
                maxBias = (frontObjectBound.leftBound > maxBias) ?
                    frontObjectBound.leftBound : maxBias;
                newBiasLargeEnough = (i == mNodeContext.GetSideObstacles().size() - 1 &&
                    mNodeContext.GetBias() - 0.5f * CAR_WIDTH - maxBias > 0.7 &&
                    mNodeContext.GetBias() - 0.5f * CAR_WIDTH != maxBias) ? false : true;
            }

            if (mNodeContext.GetCheckRightArea())
            {
                maxBias = (frontObjectBound.rightBound < maxBias) ?
                    frontObjectBound.rightBound : maxBias;
                newBiasLargeEnough = (i == mNodeContext.GetSideObstacles().size() - 1 &&
                    maxBias - (mNodeContext.GetBias() + 0.5f * CAR_WIDTH) > 1.0 &&
                    maxBias != mNodeContext.GetBias() + 0.5f * CAR_WIDTH) ? false : true;
            }
        }

        if (initialBiasCondition || newBiasLargeEnough)
            biasNeedToChange = true;

        return biasNeedToChange;
    }

    bool genBiasAction::IsNewPathSafe(const float bias)
    {
        bool newPathIsSafe = true;
        ObstacleAvoidance OA;
        Core CORE;
        float rollInDistance = 10.0f * std::sqrt(mContext->vehPosSD.speed) + 10.0f;
        PointsSD unitPath = OA.UnitPathGenerator(static_cast<int>(rollInDistance));

        PointsSD newLocalPath = OA.PathGenerator(
            unitPath,
            mContext->vehPosSD.s,
            mContext->vehPosSD.d,
            rollInDistance,
            bias,
            std::max(50 - static_cast<int>(rollInDistance), 0));

        for (int i = 0; i < mNodeContext.GetSideObstacles().size(); i++)
        {
            ObjectSD points;
            std::vector<float> objPoints;
            for (int j = 0; j < mNodeContext.GetSideObstacles()[i].s.size(); j++)
            {
                 objPoints = CORE.GetFrenet(
                     mNodeContext.GetSideObstacles()[i].s[j],
                     mNodeContext.GetSideObstacles()[i].d[j],
                     0,
                     newLocalPath.s,
                     newLocalPath.d,
                     0,
                     newLocalPath.s.size());

                if (objPoints[1] >= -0.5 * CAR_WIDTH - 0.5 &&
                    objPoints[1] <= 0.5 * CAR_WIDTH + 0.5)
                    newPathIsSafe = false;
            }
        }

        for (int i = 0; i < mNodeContext.GetOnRoadObstacles().size(); i++)
        {
            ObjectSD points;
            std::vector<float> objPoints;
            for (int j = 0; j < mNodeContext.GetOnRoadObstacles()[i].s.size(); j++)
            {
                 objPoints = CORE.GetFrenet(
                     mNodeContext.GetOnRoadObstacles()[i].s[j],
                     mNodeContext.GetOnRoadObstacles()[i].d[j],
                     0,
                     newLocalPath.s,
                     newLocalPath.d,
                     0,
                     newLocalPath.s.size());

                if (objPoints[1] >= -0.5 * CAR_WIDTH - 0.5 &&
                    objPoints[1] <= 0.5 * CAR_WIDTH + 0.5 &&
                    (std::pow((mNodeContext.GetOnRoadObstacles()[i].relativeSpeedS),2) / 2.0f) >
                    std::abs(mNodeContext.GetOnRoadObstacles()[i].s[j] - mContext->vehPosSD.s) &&
                    (mNodeContext.GetOnRoadObstacles()[i].relativeSpeedS) < 3.0f/3.6f)
                {
                    newPathIsSafe = false;
                }
            }
        }

        return newPathIsSafe;
    }

    void genBiasAction::Visualization()
    {
        visualization_msgs::MarkerArray markerList;

        int markerId = 100;
        for (const auto & object : mNodeContext.GetSideObstacles())
        {
            visualization_msgs::Marker lineMarker = InitLineMarker(
                {1.0, 1.0, 1.0});
            lineMarker.header.frame_id = "map";
            lineMarker.id = markerId ++;

            const auto objXY = mContext->objLiRarXY.find(object.id);
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


        visualization_msgs::Marker lineMarker = InitLineMarker(
            {1.0, 0.0, 0.0});
        lineMarker.header.frame_id = "map";
        lineMarker.id = markerId ++;
        const auto objXY = mContext->objLiRarXY.find(mContext->OAmMaxBiasId);
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


        mContext->pubActionMarkers.publish(markerList);
    }
}
