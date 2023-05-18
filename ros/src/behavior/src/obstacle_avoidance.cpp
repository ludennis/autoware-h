#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <vector>
#include <behavior/obstacle_avoidance.h>

namespace Behavior
{
static const float VEHICLE_WIDTH = 1.7f;
static const float VEHICLE_LENGTH = 4.0f;
static const float SPEED_RANGE = 1.0f / 3.6f;
static const float SAFE_BOUNDARY_WIDTH = 1.2f;
static const float MIN_DRIVING_SPEED = 5.0f / 3.6f;
static const float MAX_DRIVING_SPEED = 15.0f / 3.6f;
static const float DISTANCE_THRESHOLD_FOR_LONG_WAITING = 10.0f;
static const float PATH_END_CHECKING_RANGE = 0.3f;

ObstacleAvoidance::ObstacleAvoidance()
    : mObstacleStayTime(0)
    , mTargetD(0)
    , mWaitingTime(0)
{}


template <typename T>
static inline T Sigmoid(const T & x)
{
    return static_cast<T>(1.0) / (static_cast<T>(1.0) + std::exp(-x));
}

PointsSD ObstacleAvoidance::UnitPathGenerator(const int pointsResolution)
{
    PointsSD unitPath;
    for (int i = 0; i < pointsResolution + 1; i++)
    {
        float bias = Sigmoid(
                8.0 * (static_cast<float>(i) -
                static_cast<float>(pointsResolution) / 2) /
                static_cast<float>(pointsResolution));
        unitPath.s.push_back(static_cast<float>(i) /
            static_cast<float>(pointsResolution));
        unitPath.d.push_back(bias);
    }
    return unitPath;
}

PointsSD ObstacleAvoidance::PathGenerator(
    PointsSD unitPath,
    const float vehiclePositionS,
    const float vehiclePositionD,
    const float rollInDistance,
    const float bias,
    const int extraPointTime)
{
    PointsSD preGeneratePath;

    for (int i = 0; i < unitPath.s.size(); i++)
    {
        preGeneratePath.s.push_back(
            rollInDistance * unitPath.s[i] + vehiclePositionS);
        preGeneratePath.d.push_back(bias * unitPath.d[i] + vehiclePositionD);
    }

    const float endS = preGeneratePath.s.back();
    const float endD = preGeneratePath.d.back();

    for (int i = 0; i < extraPointTime; i++)
    {
        preGeneratePath.s.push_back(endS + i + 1);
        preGeneratePath.d.push_back(endD);
    }

    return preGeneratePath;
}

bool ObstacleAvoidance::PathEndChecking(
    const float targetD,
    std::vector<float> & wayPointsD)
{
    const float distance = std::abs(targetD - wayPointsD.back());
    return (distance < PATH_END_CHECKING_RANGE);
}

std::vector<ObjectSD> ObstacleAvoidance::SelectCheckingObject(
    const float upperbBound,
    const float lowerBound,
    std::vector<ObjectSD> & objectsListSD)
{
    std::vector<ObjectSD> objNeedChecking;
    bool objInRangeHint;

    for (int i = 0; i < objectsListSD.size(); i++)
    {
        objInRangeHint = false;
        for (int j = 0; j < objectsListSD[i].s.size(); j++)
        {
            if ((objectsListSD[i].s[j] < upperbBound &&
                objectsListSD[i].s[j] > lowerBound) &&
                !objInRangeHint)
            {
                objInRangeHint = true;
            }
        }
        if (objInRangeHint)
        {
            objNeedChecking.push_back(objectsListSD[i]);
        }
    }
    return objNeedChecking;
}

ObjOnPath ObstacleAvoidance::ObjectInRange(const float safeBoundaryWidth,
    std::vector<ObjectSD> & objListInPathCoordinate)
{
    ObjOnPath objectOnPath;
    objectOnPath.onPath = false;
    objectOnPath.index = -999;
    float distance = 100;

    for (int i = 0; i < objListInPathCoordinate.size(); i++)
    {
        int sameSideSum = 0;
        for (int j = 0; j < objListInPathCoordinate[i].d.size(); j++)
        {
            if (objListInPathCoordinate[i].d[j] > safeBoundaryWidth)
            {
                sameSideSum = sameSideSum + 1;
            }
            if (objListInPathCoordinate[i].d[j] < -safeBoundaryWidth)
            {
                sameSideSum = sameSideSum - 1;
            }
        }

        if (std::abs(sameSideSum) != objListInPathCoordinate[i].d.size())
        {
            if (objListInPathCoordinate[i].s[0] < distance)
            {
                distance = objListInPathCoordinate[i].s[0];
                objectOnPath.index = i;
            }
        }
    }
    if (objectOnPath.index != -999)
    {
        objectOnPath.onPath = true;
    }
    return objectOnPath;
}

ObjOnPath ObstacleAvoidance::PathWithCollisionChecking(
    std::vector<float> & pathLineS,
    std::vector<float> & pathLineD,
    std::vector<ObjectSD> & objNeedChecking,
    const float safeBoundaryWidth)
{
    std::vector<ObjectSD> objListInPathCoordinate;
    ObjOnPath objectOnPath;
    objectOnPath.onPath = false;
    objectOnPath.index = -999;

    if (objNeedChecking.size() == 0)
    {
        return objectOnPath;
    }
    else
    {
        for (int i = 0; i < objNeedChecking.size(); i++)
        {
            ObjectSD points;
            std::vector<float> objPoints;
            for (int j = 0; j < objNeedChecking[i].s.size(); j++)
            {
                // objPoints = GetFrenet(
                //     objNeedChecking[i].s[j],
                //     objNeedChecking[i].d[j],
                //     0,
                //     pathLineS,
                //     pathLineD,
                //     0,
                //     pathLineS.size());
                points.s.push_back(0.0);
                points.d.push_back(0.0);
            }
            objListInPathCoordinate.push_back(points);
        }

        objectOnPath =
            ObjectInRange(safeBoundaryWidth, objListInPathCoordinate);
        return objectOnPath;
    }
}

float ObstacleAvoidance::WaitingTimeGenerator(
    const float vehicleSpeed,
    const float vehiclePositionS,
    const float rollInDistance,
    const float objPositionS)
{
    float waitingTime = 1.0f;
    float objDistance = objPositionS - vehiclePositionS;

    if (vehicleSpeed < MIN_DRIVING_SPEED)
    {
        waitingTime = 10.0f;
    }
    else
    {
        if (objDistance <= DISTANCE_THRESHOLD_FOR_LONG_WAITING)
        {
            waitingTime = 10.0f;
        }
        else if(objDistance > DISTANCE_THRESHOLD_FOR_LONG_WAITING)
        {
            waitingTime = (objDistance - DISTANCE_THRESHOLD_FOR_LONG_WAITING) /
                MAX_DRIVING_SPEED;
        }
    }
    return waitingTime;
}

int ObstacleAvoidance::ExtraPointNumber(
    const float rollInDistance,
    const float vehicleSpeed)
{
    int extraPointNum = 0;

    if (rollInDistance < 50)
    {
        extraPointNum = static_cast<int>(
            std::min(50 - rollInDistance, vehicleSpeed * 3.0f + 5.0f));
    }
    return extraPointNum;
}

ObjRectBound ObstacleAvoidance::FindObjBound(ObjectSD objList)
{
    std::vector<float> vectorS;
    std::vector<float> vectorD;
    ObjRectBound objRectBound;

    for (int i = 0; i < objList.s.size(); i++)
    {
        vectorS.push_back(objList.s[i]);
        vectorD.push_back(objList.d[i]);
    }
    auto tmpSBound =
        std::minmax_element(std::begin(vectorS), std::end(vectorS));
    auto tmpDBound =
        std::minmax_element(std::begin(vectorD), std::end(vectorD));
    objRectBound.minS = *tmpSBound.first;
    objRectBound.maxS = *tmpSBound.second;
    objRectBound.rightBound = *tmpDBound.first;
    objRectBound.leftBound = *tmpDBound.second;

    return objRectBound;
}

int ObstacleAvoidance::FindLaneBias(
    const float PositionD,
    const float roadWidth)
{
    if (roadWidth <= 0)
    {
        return 0;
    }
    const int LaneBias = floor(( floor(PositionD / (roadWidth / 2)) + 1) / 2);
    return LaneBias;
}

std::vector<bool> ObstacleAvoidance::ExtraSpaceHint(
    const int laneNum, const int laneId)
{
    const bool ableToCrossRight = laneNum > 1 && laneId < laneNum -1;
    const bool ableToCrossLeft = laneNum > 1 && laneId > 0;
    return {ableToCrossLeft, ableToCrossRight};
}

std::vector<bool> ObstacleAvoidance::LaneLineHint(
    std::vector<LaneType> & laneType)
{
    const bool ableToCrossRight =
        laneType[0] == LaneType::WHITE_DASHED_LINE ||
        laneType[0] == LaneType::YELLOW_DASHED_LINE;
    const bool ableToCrossLeft =
        laneType[1] == LaneType::WHITE_DASHED_LINE ||
        laneType[1] == LaneType::YELLOW_DASHED_LINE;
    return {ableToCrossLeft, ableToCrossRight};
}

float ObstacleAvoidance::BiasGenerator(
    PointsSD unitPath,
    const float biasResolution,
    const float vehiclePositionS,
    const float vehiclePositionD,
    const float rollInDistance,
    const int searchingStartD,
    const int searchingEndD,
    std::vector<ObjectSD> & frontObjects,
    const int distanceTolerance,
    const int extraPointTime)
{
    float biasToUse;

    int count = 0;
    if (searchingStartD > searchingEndD)
    {
        for (int i = searchingStartD; i > searchingEndD; i--)
        {
            biasToUse =  static_cast<float>(i) / biasResolution;

            PointsSD preGeneratePath = PathGenerator(
                unitPath, vehiclePositionS, vehiclePositionD,
                rollInDistance, biasToUse, extraPointTime);

            ObjOnPath objectInLocalMotion = PathWithCollisionChecking(
                preGeneratePath.s,
                preGeneratePath.d,
                frontObjects,
                SAFE_BOUNDARY_WIDTH);
            if (!objectInLocalMotion.onPath)
            {
                count ++;
                if (count > distanceTolerance)
                {
                    return biasToUse + vehiclePositionD;
                }
            }
        }
    }
    else
    {
        for (int i = searchingStartD; i < searchingEndD; i++)
        {
            biasToUse =  static_cast<float>(i) / biasResolution;

            PointsSD preGeneratePath = PathGenerator(
                unitPath, vehiclePositionS, vehiclePositionD,
                rollInDistance, biasToUse, extraPointTime);

            ObjOnPath objectInLocalMotion = PathWithCollisionChecking(
                preGeneratePath.s,
                preGeneratePath.d,
                frontObjects,
                SAFE_BOUNDARY_WIDTH);
            if (!objectInLocalMotion.onPath)
            {
                count ++;
                if (count > distanceTolerance)
                {
                    return biasToUse + vehiclePositionD;
                }
            }
        }
    }
    return 100.0f;
}

void ObstacleAvoidance::GenMeterWaypoint(
    const float vehiclePositionS,
    const std::vector<float> & wayPointsS,
    const std::vector<float> & wayPointsD,
    std::vector<float> & wayPointsMeterS,
    std::vector<float> & wayPointsMeterD)
{
    std::vector<float> wayPointsFakeD(wayPointsS.size(),0.0f);
    int closestPathIndex = ClosestWaypoint(
        vehiclePositionS,
        0,
        wayPointsS,
        wayPointsFakeD,
        0,
        wayPointsS.size());
    wayPointsMeterS.push_back(wayPointsS[closestPathIndex]);
    wayPointsMeterD.push_back(wayPointsD[closestPathIndex]);
    for (int i = closestPathIndex + 1; i < wayPointsS.size(); i++)
    {
        if (wayPointsS[i] - wayPointsMeterS.back() >= 1)
        {
            wayPointsMeterS.push_back(wayPointsS[i]);
            wayPointsMeterD.push_back(wayPointsD[i]);
        }
    }
}

float ObstacleAvoidance::FinalPathBiasGenerator(
    const float frontObjectBound,
    const bool extraSpace,
    const bool lineCanCross,
    const float laneBias,
    const float roadWidth,
    const bool directionHint,
    PointsSD unitPath,
    const float biasResolution,
    const float vehiclePositionS,
    const float vehiclePositionD,
    const float rollInDistance,
    std::vector<ObjectSD> & frontObjects,
    const int distanceTolerance,
    const int extraPointNum)
{
    float pathBias;
    int searchingStartD;
    int searchingEndD;

    int direction = static_cast<int>(
        (static_cast<float>(directionHint) - 0.5) / 0.5);

    searchingStartD = static_cast<int>(
        (frontObjectBound - vehiclePositionD) * biasResolution);

    if (extraSpace && lineCanCross)
    {
        searchingEndD = static_cast<int>(((laneBias + direction) *
            roadWidth - vehiclePositionD) * biasResolution);
    }
    else
    {
        searchingEndD = static_cast<int>(
            ((laneBias + 0.5 * direction) * roadWidth -
            0.5 * direction * VEHICLE_WIDTH - vehiclePositionD) * biasResolution);
    }

    if (direction * searchingStartD > direction * searchingEndD)
    {
        pathBias = 100.0;
    }
    else
    {
        pathBias = BiasGenerator(
            unitPath,
            biasResolution,
            vehiclePositionS,
            vehiclePositionD,
            rollInDistance,
            searchingStartD,
            searchingEndD,
            frontObjects,
            distanceTolerance,
            extraPointNum);
    }
    return pathBias;
}

float ObstacleAvoidance::SelectBias(
    const float leftPathBias,
    const float rightPathBias,
    const float targetD)
{
    float setBias = 100;

    if (leftPathBias != 100)
    {
        setBias = leftPathBias;

        float leftBiasToTargetD = std::abs(targetD - leftPathBias);
        float rightBiasToTargetD = std::abs(targetD - rightPathBias);
        if (rightBiasToTargetD < leftBiasToTargetD && rightPathBias != 100)
        {
            setBias = rightPathBias;
        }
    }
    else if (leftPathBias == 100 && rightPathBias != 100)
    {
        setBias = rightPathBias;
    }
    return setBias;
}

BehaviorOutput ObstacleAvoidance::SpeedGenerator(
    BehaviorOutput stateVector,
    std::vector<LaneType> & laneType,
    const float vehicleSpeed)
{
    const float bumpPassingSpeed = 5.0f / 3.6f;
    const float curvePassingSpeed = 7.0f / 3.6f;
    const float minDrivingSpeed = 5.0f / 3.6f;

    if (stateVector.hint == true)
    {
        if (laneType[2] == LaneType::LONGER_BUMP ||
            laneType[2] == LaneType::SHORTER_BUMP)
        {
            stateVector.update_terminal_speed = true;
            stateVector.terminal_speed = bumpPassingSpeed;
            stateVector.update_terminal_distance = true;
            stateVector.terminal_distance = 15.0;
        }
        else
        {
            stateVector.update_terminal_speed = true;
            stateVector.terminal_speed = vehicleSpeed;
            stateVector.update_terminal_distance = true;
            stateVector.terminal_distance = 0.0f;
        }

        if (stateVector.terminal_speed < minDrivingSpeed &&
            stateVector.update_terminal_speed == true)
        {
            stateVector.terminal_speed = minDrivingSpeed;
            stateVector.update_terminal_distance = true;
            stateVector.terminal_distance = 0.0f;
        }
    }
    return stateVector;
}

BehaviorOutput ObstacleAvoidance::StateFunction(
    State behavior,
    const float lastClosestPathPoint,
    const float searchingRange,
    const float vehiclePositionS,
    const float vehiclePositionD,
    std::vector<float> & globalPathS,
    std::vector<float> & globalCrossS,
    std::vector<float> & wayPointsS,
    std::vector<float> & wayPointsD,
    VehPosXY & vehiclePosition,
    const float rollInFactor,
    std::vector<ObjectSD> & objectsListSD,
    std::vector<float> & roadWidth,
    std::vector<LaneType> & laneType,
    const int laneNum,
    const int laneId)
{
    BehaviorOutput returnAvoidance;
    returnAvoidance.hint = 0;
    returnAvoidance.numLevel = 0;
    returnAvoidance.terminal_distance = 0;
    returnAvoidance.update_num_level = false;
    returnAvoidance.update_dev   = false;
    returnAvoidance.update_bias  = false;
    returnAvoidance.update_terminal_speed = false;
    returnAvoidance.update_terminal_distance = true;

    float rollInDistance =  vehiclePosition.speed * rollInFactor + 10;
    PointsSD unitPath = UnitPathGenerator(static_cast<int>(rollInDistance));

    std::vector<bool> extraspace = ExtraSpaceHint(laneNum, laneId);
    bool leftExtraspace = extraspace[0];
    bool rightExtraspace = extraspace[1];

    std::vector<bool> sideCanCross = LaneLineHint(laneType);
    bool leftLineCanCross = sideCanCross[0];
    bool rightLineCanCross = sideCanCross[1];

    std::vector<float> fakeGlobalPathD(globalPathS.size(), 0);
    int indexOfClosestGlobalPathS =
        ClosestWaypoint(
            vehiclePositionS,
            0,
            globalPathS,
            fakeGlobalPathD,
            lastClosestPathPoint,
            searchingRange);
    int laneBias =
        FindLaneBias(vehiclePositionD, roadWidth[indexOfClosestGlobalPathS]);


    if(behavior != State::LANE_FOLLOW &&
        behavior != State::LANE_CHANGE)
    {
        return returnAvoidance;
    }

    if(behavior == State::LANE_CHANGE)
    {
        float targetD = roadWidth[indexOfClosestGlobalPathS] *
            FindLaneBias(wayPointsD.back(),
            roadWidth[indexOfClosestGlobalPathS]);
        if (std::abs(targetD - mTargetD) <=
            roadWidth[indexOfClosestGlobalPathS])
        {
            mTargetD = targetD;
        }
    }

    std::vector<ObjectSD> frontObjects = SelectCheckingObject(
        vehiclePositionS + 40.0f,
        vehiclePositionS + 5.0f,
        objectsListSD);

    std::vector<ObjectSD> backObjects = SelectCheckingObject(
        vehiclePositionS + 5.0f,
        vehiclePositionS - 25.0f,
        objectsListSD);

    std::vector<float> wayPointsMeterS;
    std::vector<float> wayPointsMeterD;
    std::vector<float> wayPointsFakeD(wayPointsS.size(),0.0f);
    int closestPathIndex = ClosestWaypoint(
        vehiclePositionS,
        0,
        wayPointsS,
        wayPointsFakeD,
        0,
        wayPointsS.size());
    wayPointsMeterS.push_back(wayPointsS[closestPathIndex]);
    wayPointsMeterD.push_back(wayPointsD[closestPathIndex]);
    for (int i = closestPathIndex + 1; i < wayPointsS.size(); i++)
    {
        if (wayPointsS[i] - wayPointsMeterS.back() >= 1)
        {
            wayPointsMeterS.push_back(wayPointsS[i]);
            wayPointsMeterD.push_back(wayPointsD[i]);
        }
    }

    ObjOnPath objectInLocalMotion = PathWithCollisionChecking(
        wayPointsMeterS,
        wayPointsMeterD,
        frontObjects,
        SAFE_BOUNDARY_WIDTH);

    bool vehicleOnTargetLane = PathEndChecking(mTargetD, wayPointsD);
    float biasResolution = 10.0;
    int searchingStartD = 0;
    int searchingEndD = 0;
    int extraPointNum =
        ExtraPointNumber(rollInDistance, vehiclePosition.speed);
    bool staticObjectInFront = false;

    if (objectInLocalMotion.onPath)
    {
        ROS_INFO("FRONT OBJECT ID: %f",
            frontObjects[objectInLocalMotion.index].id);

        ObjRectBound frontObjectBound =
            FindObjBound(frontObjects[objectInLocalMotion.index]);

        if (mWaitingTime == 0)
        {
            mWaitingTime = WaitingTimeGenerator(
                vehiclePosition.speed,
                vehiclePositionS,
                rollInDistance,
                frontObjectBound.minS);
            mObstacleStayTime = ros::Time::now().toSec();
        }

        ROS_INFO("WAITING TIME: %f", mWaitingTime);
        ROS_INFO("OBJECT STATIC TIME: %f",
            (ros::Time::now().toSec() - mObstacleStayTime));


        if (std::abs(
            frontObjects[objectInLocalMotion.index].speedS) < 1.0f &&
            std::abs(
            frontObjects[objectInLocalMotion.index].speedD) < 1.0f)
        {
            if (ros::Time::now().toSec() - mObstacleStayTime > mWaitingTime)
            {
                if (frontObjectBound.minS - vehiclePositionS <= rollInDistance)
                {
                    staticObjectInFront = true;
                    mWaitingTime = 0;
                    mObstacleStayTime = ros::Time::now().toSec();
                }
                else
                {
                    staticObjectInFront = false;
                }
            }
        }
        else
        {
            mObstacleStayTime = ros::Time::now().toSec();
            mWaitingTime = 0;
            staticObjectInFront = false;
        }

        if (staticObjectInFront)
        {
            if (std::abs(vehiclePositionD - mTargetD) > 1.0f)
            {
                return returnAvoidance;
            }

            searchingStartD = static_cast<int>(
                (frontObjectBound.leftBound - vehiclePositionD) *
                biasResolution);

            if (leftExtraspace && leftLineCanCross)
            {
                searchingEndD = static_cast<int>(((laneBias + 1) *
                    roadWidth[indexOfClosestGlobalPathS] -vehiclePositionD) *
                    biasResolution);
            }
            else
            {
                searchingEndD = static_cast<int>(
                    ((laneBias + 0.5) * roadWidth[indexOfClosestGlobalPathS] -
                    0.5 * VEHICLE_WIDTH - vehiclePositionD) * biasResolution);
            }

            float leftPathBias;
            if (searchingStartD > searchingEndD)
            {
                leftPathBias = 100.0;
            }
            else
            {
                leftPathBias = BiasGenerator(
                    unitPath,
                    biasResolution,
                    vehiclePositionS,
                    vehiclePositionD,
                    rollInDistance,
                    searchingStartD,
                    searchingEndD,
                    frontObjects,
                    3,
                    extraPointNum);
            }
            bool avoidByLeft = (leftPathBias != 100);

            searchingStartD = static_cast<int>(
                (frontObjectBound.rightBound - vehiclePositionD) *
                biasResolution);
            if (rightExtraspace && rightLineCanCross)
            {
                searchingEndD = static_cast<int>(((laneBias - 1) *
                    roadWidth[indexOfClosestGlobalPathS] - vehiclePositionD) *
                    biasResolution);
            }
            else
            {
                searchingEndD = static_cast<int>(
                    ((laneBias - 0.5) * roadWidth[indexOfClosestGlobalPathS] +
                    0.5 * VEHICLE_WIDTH - vehiclePositionD) * biasResolution);
            }

            float rightPathBias;
            if (searchingStartD < searchingEndD)
            {
                rightPathBias = 100.0;
            }
            else
            {
                rightPathBias = BiasGenerator(
                    unitPath,
                    biasResolution,
                    vehiclePositionS,
                    vehiclePositionD,
                    rollInDistance,
                    searchingStartD,
                    searchingEndD,
                    frontObjects,
                    3,
                    extraPointNum);
            }
            bool avoidByRight = (rightPathBias != 100);

            if (avoidByLeft)
            {
                returnAvoidance.hint = 1;
                returnAvoidance.update_bias  = true;
                returnAvoidance.bias  = leftPathBias;

                float leftBiasToTargetD = std::abs(mTargetD - leftPathBias);
                float rightBiasToTargetD = std::abs(mTargetD - rightPathBias);
                if (rightBiasToTargetD < leftBiasToTargetD && avoidByRight)
                {
                    returnAvoidance.bias  = rightPathBias;
                }
            }
            else if (!avoidByLeft && avoidByRight)
            {
                returnAvoidance.hint = 1;
                returnAvoidance.update_bias  = true;
                returnAvoidance.bias  = rightPathBias;
            }

            returnAvoidance = SpeedGenerator(
                returnAvoidance, laneType, vehiclePosition.speed);

            if (returnAvoidance.hint)
            {
                std::vector<float> fakeTargetLanePathS;
                std::vector<float> fakeTargetLanePathD;
                for (int i = 0; i < (20 + static_cast<int>(rollInDistance)); i++)
                {
                    fakeTargetLanePathS.push_back(
                        vehiclePositionS - 20.0 +  static_cast<float>(i));
                    fakeTargetLanePathD.push_back(returnAvoidance.bias);
                }

                ObjOnPath targetLaneBehindObject = PathWithCollisionChecking(
                    fakeTargetLanePathS,
                    fakeTargetLanePathD,
                    objectsListSD,
                    1.0);

                if (targetLaneBehindObject.onPath)
                {
                    if (frontObjects[objectInLocalMotion.index].id !=
                        objectsListSD[targetLaneBehindObject.index].id)
                    {
                        returnAvoidance.hint = 0;
                        returnAvoidance.update_bias  = false;
                    }
                }
            }

            return returnAvoidance;
        }
        else
        {
            return returnAvoidance;
        }
    }
    else
    {
        mObstacleStayTime = ros::Time::now().toSec();
        mWaitingTime = 0;

        if (vehicleOnTargetLane)
        {
            return returnAvoidance;
        }
        else
        {
            if (vehiclePositionD > mTargetD)
            {
                searchingStartD = static_cast<int>(
                    (mTargetD + 0.1 - vehiclePositionD) * biasResolution);
                searchingEndD = static_cast<int>(
                    (mTargetD - 0.1 - vehiclePositionD) * biasResolution);
            }
            else
            {
                searchingStartD = static_cast<int>(
                    (mTargetD - 0.1 - vehiclePositionD) * biasResolution);
                searchingEndD = static_cast<int>(
                    (mTargetD + 0.1 - vehiclePositionD) * biasResolution);
            }

            float goBackBias = BiasGenerator(
                unitPath,
                biasResolution,
                vehiclePositionS,
                vehiclePositionD,
                rollInDistance,
                searchingStartD,
                searchingEndD,
                frontObjects,
                1,
                extraPointNum);

            bool goBack = (goBackBias != 100);
            std::vector<float> fakeTargetLanePathS;
            std::vector<float> fakeTargetLanePathD;
            for (int i = 0; i < (20 + static_cast<int>(rollInDistance)); i++)
            {
                fakeTargetLanePathS.push_back(
                    vehiclePositionS - 20.0 +  static_cast<float>(i));
                fakeTargetLanePathD.push_back(goBackBias);
            }
            ObjOnPath targetLaneBehindObject = PathWithCollisionChecking(
                fakeTargetLanePathS,
                fakeTargetLanePathD,
                objectsListSD,
                1.0);

            if (goBack)
            {
                if (targetLaneBehindObject.onPath)
                {
                    int objectIndex = targetLaneBehindObject.index;
                    ObjRectBound behindObjectBound =
                        FindObjBound(objectsListSD[objectIndex]);

                    if (vehiclePositionS - behindObjectBound.maxS > 0)
                    {
                        returnAvoidance.hint = true;
                        returnAvoidance.update_bias  = true;
                        returnAvoidance.bias  = goBackBias;
                    }
                }
                else
                {
                    returnAvoidance.hint = true;
                    returnAvoidance.update_bias  = true;
                    returnAvoidance.bias  = goBackBias;
                }
                returnAvoidance = SpeedGenerator(
                    returnAvoidance, laneType, vehiclePosition.speed);
            }
            return returnAvoidance;
        }
    }
}
}
