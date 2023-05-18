#include <behavior/Avoidance/hsr_obstacle_on_roadside_condition.h>
#include <string>
#include <mutex>

#define OA_DEBUG
static const float CAR_WIDTH = 2.0f;

namespace Behavior
{
    objOnRoadsideCondition::objOnRoadsideCondition(
        std::string name,
        const std::shared_ptr<BehaviorOutput> & nodeCommand,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context,
        hsrAvoidanceSequence & nodeContext)
        : ConditionNode::ConditionNode(name)
        , mNodeCommand(nodeCommand)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
        , mNodeContext(nodeContext)
    {

    }

    BT::ReturnStatus objOnRoadsideCondition::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);
        std::vector<ObjectSD> frontObstacles = FrontObstaclesInLocalPath();
        SeperateObstacles(frontObstacles);

#ifdef OA_DEBUG
        ROS_INFO_STREAM("The Condition " << get_name() << " has succeeded");
#endif
        return BT::SUCCESS ;
    }

    static inline float GetPrincipalAngle(const float & angle)
    {
        float angleMod = std::fmod(angle, 2.0f * M_PI);
        if (std::signbit(angleMod))
            angleMod += 2.0f * M_PI;
        return angleMod;
    }

    std::vector<ObjectSD> objOnRoadsideCondition::FrontObstaclesInLocalPath()
    {
        std::vector<ObjectSD> frontObstacles;
        float leftBound = std::max(2.0, mContext->vehPosSD.d + 0.5 * CAR_WIDTH + 0.5);
        float rightBound = std::min(-2.0, mContext->vehPosSD.d - 0.5 * CAR_WIDTH - 0.5);

        for (int i = 0; i < mContext->objLiRarSD.size(); i++)
        {
            bool notFindObjectInFront = true;
            for (int j = 0; j < mContext->objLiRarSD[i].s.size() && notFindObjectInFront; j++)
            {
                if ((mContext->objLiRarSD[i].s[j] - mContext->vehPosSD.s >= 0.0f &&
                    mContext->objLiRarSD[i].s[j] - mContext->vehPosSD.s <= 50.0f) &&
                    mContext->objLiRarSD[i].s[j] - mContext->vehPosSD.s <= mContext->globalPathS.back() &&
                    mContext->objLiRarSD[i].d[j] <= leftBound &&
                    mContext->objLiRarSD[i].d[j] >= rightBound)
                {
                    frontObstacles.push_back(mContext->objLiRarSD[i]);
                    notFindObjectInFront = false;
                }
            }
        }
        return frontObstacles;
    }

    void objOnRoadsideCondition::SeperateObstacles(
        std::vector<ObjectSD> & frontObstacles)
    {
        bool condition = false;
        bool condition2 = false;
        bool condition3 = false;
        std::vector<ObjectSD> sideObstacles;
        std::vector<ObjectSD> onRoadObstacles;
        ObstacleAvoidance OA;
        for (int i = 0; i < frontObstacles.size(); i++)
        {
            ObjRectBound frontObjectBound = OA.FindObjBound(frontObstacles[i]);
            condition =  ObstacleIsStatic(frontObstacles[i]) &&
                FindObjSideRatio(frontObstacles[i]) < 0.5f;
            condition2 = (frontObjectBound.rightBound > 0.5f && frontObjectBound.rightBound < 2.65f);
            condition3 = frontObjectBound.rightBound < -2.0f &&
                frontObstacles[i].id <= 400;

            if (condition && !condition3)
                sideObstacles.push_back(frontObstacles[i]);
            else if(condition2 && !condition3)
                onRoadObstacles.push_back(frontObstacles[i]);
        }

        mNodeContext.SetSideObstacles(sideObstacles);
        mNodeContext.SetOnRoadObstacles(onRoadObstacles);
    }


    float objOnRoadsideCondition::FindObjSideRatio(const ObjectSD frontObject)
    {
        ObstacleAvoidance OA;
        ObjRectBound frontObjectBound = OA.FindObjBound(frontObject);
        const float halfLaneWidth = 1.3f;
        if (frontObject.id > 999 &&
            frontObjectBound.leftBound - frontObjectBound.rightBound < 1.0f)
            return 1.0;


        if (std::signbit(
            frontObjectBound.rightBound * frontObjectBound.leftBound))
        {
            if (frontObjectBound.leftBound < 0.6f &&
                std::abs(frontObjectBound.leftBound / frontObjectBound.rightBound) < 0.33f)
                return 0.0f;
            else
                return 1.0f;
        }
        else
        {
            if (frontObjectBound.rightBound < 0.0f)
                return 0.0f;
            else
                return 1.0f;
        }
    }

    bool objOnRoadsideCondition::ObstacleIsStatic(const ObjectSD frontObject)
    {
        ObjectXY objXY = mContext->objLiRarXY[frontObject.id];
        float objX = (objXY.centerX - mContext->vehPosXY.x) *
                std::cos(mContext->vehPosXY.heading) -
            (objXY.centerY - mContext->vehPosXY.y) *
                std::sin(mContext->vehPosXY.heading);
        float objY = (objXY.centerX - mContext->vehPosXY.x) *
                std::sin(mContext->vehPosXY.heading) +
            (objXY.centerY - mContext->vehPosXY.y) *
                std::cos(mContext->vehPosXY.heading);

        std::vector<float> distance;
        for (int i = 0; i < mContext->relativeWayPointsX.size(); i ++)
        {
            const float diffX = mContext->relativeWayPointsX[i];
            const float diffY = mContext->relativeWayPointsY[i];
            distance.push_back(std::hypot(diffX, diffY));
        }
        int indexMatch =
            std::min_element(distance.begin(), distance.end()) -
                distance.begin();

        float objRelativeDistance = std::hypot(objX, objY);

        float rotateSpeedDirection =
            GetPrincipalAngle(std::atan2(objY, objX) + M_PI_2);

        float objRelativeSpeedX = objXY.relativeSpeedX +
            mContext->vehPosXY.speed +
            objRelativeDistance * mContext->vehPosXY.yawRate * std::cos(rotateSpeedDirection);
        float objRelativeSpeedY = objXY.relativeSpeedY +
            // mContext->vehPosXY.speed * std::sin(mContext->relativeWayPointsHeading[indexMatch]) +
            objRelativeDistance * mContext->vehPosXY.yawRate * std::sin(rotateSpeedDirection);

        if (objXY.id < 400)
        {
            objRelativeSpeedX = objXY.speedX;
            objRelativeSpeedY = objXY.speedY;
        }

        float trueSpeed = std::hypot(objRelativeSpeedX, objRelativeSpeedY);

        return trueSpeed < 5.0f / 3.6f;
    }
}
