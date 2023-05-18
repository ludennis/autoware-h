#include <behavior/lane_following.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include "trace/utils.h"
#include <behavior/core.h>
#include <visualization_msgs/Marker.h>

#define TRACE_TAG "LaneFollowCore"
#define LOG_DEBUG() printf("%s (%d)\n", __FILE__, __LINE__)
// #define _LF_DEBUG_

namespace Behavior
{

static inline float GetPrincipalAngle(const float & angle)
{
    float angleMod = std::fmod(angle, 2.0f * M_PI);
    if (std::signbit(angleMod))
        angleMod += 2.0f * M_PI;
    return angleMod;
}

template <typename T>
static inline T Clamp(const T & value, T bottom, T top)
{
    return std::max(bottom, std::min(top, value));
}

static inline void GetCenterPoint(const ObjectSD & pts, float & x, float & y)
{
    x = std::accumulate(pts.s.begin(), pts.s.end(), 0.0f);
    y = std::accumulate(pts.d.begin(), pts.d.end(), 0.0f);
    x /= static_cast<float>(pts.s.size());
    y /= static_cast<float>(pts.d.size());
}

static const float ERROR_PI = 3.14f;
static const float CURVE_THRESHOLD = 125.0f;
static const float SMALL_RADOFCURVE = 60.0f;
static const float BUFFER = 8.0f; //seconds
static const float TTC_TIME = 1.5f; //seconds
static const float SAVE_DISTANCE = 8.0f; // meter
static const float AVOID_SPEED = 7.0f / 3.6f; // m/s
static const float STOP_SPEED = 2.0f / 3.6f; // m/s
static const float SMALL_RADOFCURVE_SPEED = 7.0f / 3.6f; // m/s
static const float SPEEDUP_CONST = 0.5f / 3.6f; // m/s
static const bool BEHAVIOR_DEFAULT = false;
static const float BIG_CONST = 100000000.0f;
static const float SPEEDDOWN_CONST = 0.7f;
static const float AVOID_WIDTH = 1.3f;
static const float OBJS_FILTER = 3.0f;
static const float ACCELERATIOM = 0.25f;
static const int CATCHSIZE = 100;
static const float ACC_LOW_SPEED = 0.0f;
static const float HIGH_ACCELERATION = 0.5f;
static const float LOW_ACCELERATION = 0.5f;


LaneFollowing::LaneFollowing()
    : mState()
    , mPrevSpeed(0.0)
    , mCurveNum(0)
    , mCalCurvePoint(false)
    , mCurrentStopLine(0)
    , mCurrentBumper(0)
    , mAccObjID(-1)
{
    mState = State::INITIAL;
    mPubRvizLFObj = mLFNode.advertise<visualization_msgs::Marker>(
        "viz_LF_obj", 1, true);
}

visualization_msgs::Marker LaneFollowing::PubRvizLFObj(
    const int objID, const std::vector<ObjectXY> & objListXY,
    const float r, const float g, const float b)
{
    visualization_msgs::Marker rvizLFObj;
    visualization_msgs::Marker rvizLFObjCenterPt;
    //visualization_msgs::Marker marker_text;
    rvizLFObj.header.frame_id = "/map";
    rvizLFObj.header.stamp = ros::Time::now();
    rvizLFObj.ns = "points_and_lines";
    rvizLFObj.action = visualization_msgs::Marker::ADD;
    rvizLFObj.pose.orientation.w = 1.0;
    rvizLFObj.id = objID;
    rvizLFObj.type = visualization_msgs::Marker::LINE_STRIP;
    rvizLFObj.scale.x = 0.5;
    rvizLFObj.color.r = r;
    rvizLFObj.color.g = g;
    rvizLFObj.color.b = b;
    rvizLFObj.color.a = 1.0;
    rvizLFObj.lifetime = ros::Duration(0.1);

    if (objID > -1)
    {
        if(objID > 400)
        {

        }
        else
        {
            for (int i = 0;i < objListXY[objID].z.size(); ++i)
            {
                geometry_msgs::Point markerPt;
                markerPt.x = objListXY[objID].x[i];
                markerPt.y = objListXY[objID].y[i];
                markerPt.z = objListXY[objID].z[i];
                rvizLFObj.points.push_back(markerPt);
            }
        }
    }

    return rvizLFObj;
}

int LaneFollowing::DoPathMatching(
    const std::vector<float> & pathX,
    const std::vector<float> & pathY,
    const float x,
    const float y)
{
    std::vector<float> distance;
    for (int i = 0; i < pathX.size(); i ++)
    {
        const float diffX = pathX[i] - x;
        const float diffY = pathY[i] - y;
        distance.push_back(std::hypot(diffX, diffY));
    }
    int indexMatch =
        std::min_element(distance.begin(), distance.end()) -
            distance.begin();

    return indexMatch;
}

int LaneFollowing::ClosestObjOnLocalPath(
    const float maxSpeed,
    const std::vector<ObjectXY> & objListXY,
    const std::vector<ObjectSD> & objListSD,
    Polygon & accRoiPoly,
    const VehPosSD vehPos,
    bool & obstacleHazard,
    float & shortestDistance,
    float & frontVelocity,
    visualization_msgs::MarkerArray & LFMarkerArray,
    std::vector<ObjectXY> & ACCFrontObjs,
    ObjectXY & ACCObj)
{
    TRACE_ASSERT_THROW(objectsList.size() != 0);

    Core core;
    float shortestObjIndex = BIG_CONST;
    float lookAheadDistance = 50.0f;
    ObjectSD objectsTmp;
    bool trackFrontObj = false;
    int objID = -1;
    std::vector<int> frontObjIds;
    int laneFollowId = -1;
    if (objListXY.size() > 0)
    {
        for (int i = 0; i < objListXY.size(); ++i)
        {
            Polygon objListXYPoly = core.ObjListXYToPoly(objListXY[i]);
            OrderClockwise(objListXYPoly);
            const float occupiedArea = core.IntersectionArea(
                accRoiPoly, objListXYPoly);
            const float ObjArea = core.PolygonArea(objListXYPoly);

            const float occupiedPercentage = occupiedArea / ObjArea;

            float minObjS = 0;

            if (occupiedArea > 0.0f)
            {
                frontObjIds.push_back(objListXY[i].id);

                minObjS =
                    *std::min_element(objListSD[i].s.begin(), objListSD[i].s.end());
                float obj2VehDistance = minObjS - vehPos.s;
                if (obj2VehDistance < lookAheadDistance &&
                    obj2VehDistance > 3.0f && shortestObjIndex > obj2VehDistance)
                {
                    shortestObjIndex = obj2VehDistance;
                    shortestDistance = obj2VehDistance;
                    objectsTmp = objListSD[i];
                    laneFollowId = objListXY[i].id;
                    objID = i;
                    trackFrontObj = true;
                }
            }
        }
    }

    if (trackFrontObj)
    {
        float minObjCenterS;
        float minObjCenterD;

        frontVelocity = objectsTmp.relativeSpeedS;
// #ifdef _LF_DEBUG_
        std::cout << "Test objID = " << objectsTmp.id <<'\n';
        std::cout << "Test shortestDistance = " << shortestDistance <<'\n';
        std::cout << "Test relative frontVelocity = " << frontVelocity * 3.6f << ", "<< frontVelocity <<'\n';
// #endif
        obstacleHazard = true;

        //Concern object on gui
        mAccObjID = objectsTmp.id;
    }
    else
    {
        shortestDistance = 0;
        frontVelocity = 0;
        obstacleHazard = false;
        objID = -1;
    }
    // LFMarkerArray.markers.push_back(PubRvizLFObj(objID, objListXY, 1.0f,0.0f,0.0f));


    for (size_t i = 0; i < frontObjIds.size(); i++)
    {
        for (size_t j = 0; j < objListXY.size(); j++)
        {
            if (frontObjIds[i] == objListXY[j].id && frontObjIds[i] != laneFollowId)
            {
                ACCFrontObjs.push_back(objListXY[j]);
                break;
            }
            else if (frontObjIds[i] == objListXY[j].id && frontObjIds[i] == laneFollowId)
            {
                ACCObj = objListXY[j];
                break;
            }
        }
    }

    return objID;
}

void LaneFollowing::PIDRelease()
{
    mKp = 0;
    mKd = 0;
    mKi = 0;
    mPError = 0;
    mIError = 0;
    mDError = 0;
    mPrevSetPoint = 0;
    mAccTrigger = false;
}

void LaneFollowing::PIDInit(float kp, float ki, float kd)
{
    this->mKp = kp;
    this->mKi = ki;
    this->mKd = kd;
}

void LaneFollowing::UpdatePIDError(
    const float vehSpeed,
    const float shortestDistance,
    float & frontVelocity,
    float & deltaDistance)
{
    mMaintainingDistance = false;
    deltaDistance = shortestDistance - (SAVE_DISTANCE + TTC_TIME * vehSpeed);

    float deltaVelocity = frontVelocity;
#ifdef _LF_DEBUG_
    std::cout << "deltaVelocity = " << deltaVelocity <<'\n';
    std::cout << "deltaDistance = " << deltaDistance <<'\n';
    std::cout << "(SAVE_DISTANCE + TTC_TIME * vehSpeed) = " << (SAVE_DISTANCE + TTC_TIME * vehSpeed) <<'\n';
#endif
    if (deltaVelocity < 0 && deltaDistance < 0) mMaintainingDistance = true;

    float setControlPoint = 0.75f * deltaVelocity + 0.45f * deltaDistance;
    if (mPrevSetPoint == 0) mPrevSetPoint = setControlPoint;
    mPError = setControlPoint;
    mIError += mPError * 0.001f;
// #ifdef _LF_DEBUG_
    std::cout << "mPError = " << mPError <<'\n';
    std::cout << "mIError = " << mIError <<'\n';
// #endif
    mDError = setControlPoint - mPrevSetPoint;
    mPrevSetPoint = setControlPoint;
}

void LaneFollowing::PIDController(float & controlVelocity)
{
    std::cout << "mKp * mPError = " << mKp * mPError <<'\n';
    std::cout << "mKi * mIError = " << mKi * mIError <<'\n';
    float totalError = mKp * mPError + mKi * mIError + mKd * mDError;
    controlVelocity = totalError;
}

float LaneFollowing::ACCState(
    const float prevCmdSpeed,
    const float carSpeed,
    const float maxSpeed,
    float & shortestDistance,
    float & frontVelocity,
    const float pidPGain,
    const float pidIGain,
    const float pidDGain,
    float & deltaDistance)
{
    float controlVelocity = prevCmdSpeed;
    PIDInit(pidPGain, pidIGain, pidDGain);
    UpdatePIDError(
        carSpeed,
        shortestDistance,
        frontVelocity,
        deltaDistance);
    PIDController(controlVelocity);
    return controlVelocity;
}

void LaneFollowing::ACCOutput(
    const float prevCmdSpeed,
    const float carSpeed,
    const float maxSpeed,
    const float curveSpeed,
    const bool bumpHint,
    const bool enterCurve,
    float & shortestDistance,
    float & frontVelocity,
    const double & timeStep,
    const float pidPGain,
    const float pidIGain,
    const float pidDGain,
    float & accSpeed,
    float & controlAccelerate,
    float & deltaDistance)
{
    TRACE_ASSERT_THROW(shortestDistance != 0);

    controlAccelerate = ACCState(
        prevCmdSpeed,
        carSpeed,
        maxSpeed,
        shortestDistance,
        frontVelocity,
        pidPGain,
        pidIGain,
        pidDGain,
        deltaDistance);
    controlAccelerate = Clamp(controlAccelerate, -7.0f, 1.0f);
    accSpeed = carSpeed + controlAccelerate * timeStep;
    if (enterCurve) accSpeed = std::min(curveSpeed, accSpeed);
    accSpeed = Clamp(accSpeed, ACC_LOW_SPEED, maxSpeed);
// #ifdef _LF_DEBUG_
    std::cout << "小智 controlAccelerate = " << controlAccelerate <<'\n';
    std::cout << "timeStep = " << controlAccelerate * timeStep <<'\n';
    std::cout << "小智 coaccSpeed = " << accSpeed*3.6f <<'\n';
// #endif
}

void LaneFollowing::VelocitySetPoint(
    const float carSpeed,
    const float curveSpeed,
    const bool bumpHint,
    const bool enterCurve,
    const float maxSpeed,
    float & frontVelocity)
{
    if (bumpHint)
    {
        frontVelocity = std::min(AVOID_SPEED, frontVelocity);
    }
    else if (enterCurve)
    {
        frontVelocity = std::min(curveSpeed, frontVelocity);
    }
    else if (frontVelocity > maxSpeed)
    {
        frontVelocity = maxSpeed;
    }
}

void LaneFollowing::UpdateTrafficStopLine(
    int & currentStopLine,
    const float vehPosS,
    const int nextTrafficStopLine,
    const int trafficLightStatus)
{
    if (static_cast<float>(vehPosS - currentStopLine) > 0.0f &&
        trafficLightStatus != 1)
    {
        currentStopLine = nextTrafficStopLine;
    }
}

float LaneFollowing::ForceSpeedByDistance(
    float trafficObjIndex,
    const float targetSpeed,
    const float vehPosS,
    const float carSpeed,
    const float lastSpeedCmd,
    const float maxSpeed,
    const double timeStep)
{
    float distanceToEnd = std::max(0.0f, trafficObjIndex - vehPosS);
    float acce = -0.25f * ((std::pow(carSpeed, 2.0f) - std::pow(targetSpeed, 2.0f)) /
        (2.0f * distanceToEnd));
    float forceSpeed =
        std::max(0.0f, lastSpeedCmd + static_cast<float>(acce * timeStep));
    float terminalSpeed =
        std::sqrt(distanceToEnd * 2.0f * 0.1f + std::pow(targetSpeed, 2.0f));
    // if (carSpeed < terminalSpeed)
    // {
    //     forceSpeed = lastSpeedCmd +
    //     ((std::pow(terminalSpeed, 2.0f) - std::pow(carSpeed, 2.0f)) /
    //     (2.0f * 4.0f * 0.5f * distanceToEnd)) * timeStep;
    //     forceSpeed = std::min(forceSpeed, maxSpeed);
    // }
    if (distanceToEnd < 1.0f && distanceToEnd < carSpeed / 2.0f)
        forceSpeed = targetSpeed;

    return forceSpeed;
}

float LaneFollowing::TerminlDistance(
    const float carSpeed,
    const float terminalSpeed)
{
    float terminalDistance;
    return terminalDistance =
        std::fabs((std::pow(carSpeed, 2) - std::pow(terminalSpeed, 2))) /
        (2 * ACCELERATIOM);
}

bool LaneFollowing::CurveAvoid(
    const float vehiclePositionS,
    const std::vector<float> & globalPathS,
    const std::vector<float> & globalPathCurvature,
    const std::vector<int> & indexStartCurve,
    const std::vector<int> & indexEndCurve,
    const int curveNum,
    const float maxSpeed,
    float & curveSpeed)
{
    bool curveCheck = false;
    float minRadOfCurveture =
        *std::min_element(
            globalPathCurvature.begin() + indexStartCurve[curveNum],
            globalPathCurvature.begin() + indexEndCurve[curveNum]);
    float lateralAcc = 0.411111;
    if (vehiclePositionS > 1100.0f && vehiclePositionS < 1118.0f)
        lateralAcc = 1.511111;
    float minCurveSpeed = std::sqrt(minRadOfCurveture * lateralAcc);

    float slowDownDistance = TerminlDistance(maxSpeed, minCurveSpeed);
    float startCurvePoint =
        globalPathS[indexStartCurve[curveNum]] - slowDownDistance;

    if (vehiclePositionS > startCurvePoint &&
        vehiclePositionS < globalPathS[indexEndCurve[curveNum] - 3]) // level curve speed
    {
        curveCheck = true;

        if (vehiclePositionS > startCurvePoint &&
            vehiclePositionS < globalPathS[indexStartCurve[curveNum]] -3)
        {
            curveSpeed =
                std::sqrt(2*(globalPathS[indexStartCurve[curveNum]] - vehiclePositionS) + std::pow(minCurveSpeed,2));
        }
        else
        {
            curveSpeed = minCurveSpeed;
        }

        curveSpeed = std::min(curveSpeed, maxSpeed);
    }

    return curveCheck;
}

void LaneFollowing::CurveGetStartEnd(
    const std::vector<float> & globalPathCurvature,
    std::vector<int> & indexStartCurve,
    std::vector<int> & indexEndCurve)
{
    bool enterCurve = false;
    for (int i = 0; i < globalPathCurvature.size(); ++i)
    {
        if ((enterCurve == false) && (i != 0) &&
            globalPathCurvature[i - 1] == CURVE_THRESHOLD &&
            globalPathCurvature[i] < CURVE_THRESHOLD)
        {
            if (indexEndCurve.size() != 0 && (i - indexEndCurve.back()) < 0)
            {
                indexEndCurve.pop_back();
                enterCurve = true;
            }
            else
            {
                indexStartCurve.push_back(i);
                enterCurve = true;
            }
        }

        if (i == 0 && (globalPathCurvature[i] < CURVE_THRESHOLD) &&
            (globalPathCurvature[i+1] < CURVE_THRESHOLD))
        {
            indexStartCurve.push_back(i);
            enterCurve = true;
        }

        if ((i == (globalPathCurvature.size() - 1)) &&
            (globalPathCurvature[i] < CURVE_THRESHOLD) && enterCurve)
        {
            indexEndCurve.push_back(i);
            enterCurve = false;
        }

        if ((i != 0) && globalPathCurvature[i] == CURVE_THRESHOLD &&
            (globalPathCurvature[i - 1] < CURVE_THRESHOLD) && enterCurve)
        {
            indexEndCurve.push_back(i);
            enterCurve = false;
        }
    }
}

void LaneFollowing::CurveSpeed(
    const float carSpeed,
    SpeedCmd & LFOutput,
    const float curveSpeed)
{
    float speedupAcc = 0.5f;//((carSpeed < 10.0f/3.6f)? 0.6f:0.25f);
    float curveAcceleration = (carSpeed < curveSpeed ? speedupAcc : -0.2f);
    LFOutput.state_hint = true;
    LFOutput.speed = curveSpeed;
    LFOutput.acceleration =
        ((std::fabs(carSpeed - curveSpeed) < SPEEDUP_CONST) ? 0.0f : curveAcceleration);
}

int LaneFollowing::NextCurveIndex(
    const float vehicleS,
    const std::vector<int> & indexEndCurve,
    const std::vector<float> & globalPathS)
{
    int lower_bound = 0;
    int upper_bound = indexEndCurve.size() - 1;
    if (vehicleS > globalPathS[indexEndCurve[upper_bound]])
    {
        return -1;
    }
    else if (vehicleS < globalPathS[indexEndCurve[0]])
    {
        return 0;
    }

    while((upper_bound - lower_bound + 1) > 2)
    {
        int middle = floor((upper_bound + lower_bound) / 2);
        if (globalPathS[indexEndCurve[middle]] < vehicleS)
        {
            lower_bound = middle;
        }
        else
        {
            upper_bound = middle;
        }
    }

    for (int i = upper_bound; i >= lower_bound; --i)
    {
        if (globalPathS[indexEndCurve[i]] < vehicleS) return i+1;
    }
}

}//namespace
