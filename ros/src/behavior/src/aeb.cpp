#include <iostream>
#include <vector>
#include <cmath>
#include <behavior/aeb.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

#define _AEB_DEBUG_

namespace Behavior
{
    static const float AEB_ROI_TTC = 1.5f;
    static const float MAX_RELATIVE_TTC = 20.0f;
    static const float MIN_URGENT_RELATIVE_TTC = 0.0f;
    static const float MAX_URGENT_RELATIVE_TTC = 1.2f;
    static const float MIN_OBJ_ANGLE = 45.0f;
    static const float MIN_AEB_ROI_DISTANCE = 3.0f;
    static const float MAX_AEB_ROI_DISTANCE = 20.0f;
    static const float SAFETY_DISTANCE = 3.0f;
    static const float STOP_SPEED_CMD = 0.0f;
    static const float STOPLINE_PASS_SPEED_CMD = 2.0f / 3.6f;
    static const float AEB_RELEASE_SPEED = 1.0f / 3.6f;
    static const float AEB_RELEASE_SPEED_CMD = 2.0f / 3.6f;
    static const float MIN_OBJ_DISTANCE = 2.0f;
    static const float URGENT_OBJ_X_DISTANCE = 3.0f;
    static const float URGENT_OBJ_Y_DISTANCE = 0.7f;
    static const float URGENT_OBJ_D_DISTANCE = 0.8f;
    static const float INTERSECTION_STOPLINE_BOUNDARY = 5.0f;
    static const float INTERSECTION_X_WIDTH = 5.0f;
    static const float INTERSECTION_Y_RIGHT_BOUNDARY = -1.5f;
    static const float INTERSECTION_Y_LEFT_BOUNDARY = 2.0f;
    static const float INTERSECTION_S_BOUNDARY = 5.0f;
    static const float STOPLINE_SENSING_ROI = 15.0f;
    static const float STOPLINE_STOP_ROI = 14.0f;
    static const float STOPLINE_STOP_SPEED = 1.0f / 3.6f;
    static const double COUNTER_RESET_TIME = 0.0;
    static const double COUNTER_MAX_TIME = 1000.0;
    static const double STOPLINE_WAITING_TIME = 5.0;

    AEB::AEB()
        : mVehicleStopTime(STOPLINE_WAITING_TIME)
        , mCounterStartTime(0.0)
        , mStopLineId(-1)
        , mClosestObjId()
        , mClosestObjDistance()
    {
        mPubRvizObjInAEB = mNode.advertise<visualization_msgs::Marker>(
            "viz_AEB_obj", 1, true);
    }

    struct Point
    {
        float x;
        float y;
        Point(float x,float y)
        {
            this->x = x;
            this->y = y;
        }
    };

    float AEB::FrontRoiDistance(const float carSpeed)
    {
        float frontRoiDistance = AEB_ROI_TTC * carSpeed;
        if (frontRoiDistance < MIN_AEB_ROI_DISTANCE)
        {
            frontRoiDistance = MIN_AEB_ROI_DISTANCE;
        }
        else if (frontRoiDistance > MAX_AEB_ROI_DISTANCE)
        {
            frontRoiDistance = MAX_AEB_ROI_DISTANCE;
        }
        return frontRoiDistance;
    }

    float AEB::CountRelativeTTC(
        const float carSpeed,
        const float objectSpeed,
        const float distance)
    {
        float relativeTTC = distance / (carSpeed - objectSpeed);
        if (relativeTTC > MAX_RELATIVE_TTC)//max concern TTC
        {
            relativeTTC = MAX_RELATIVE_TTC;
        }
        return relativeTTC;
    }

    std::vector<ObjectXY> AEB::ObjRotatedXY(
        VehPosXY & vehPosXY,
        std::vector<ObjectXY> & objListXY)
    {
        std::vector<ObjectXY> objRotatedXY;
        objRotatedXY.clear();
        if (objListXY.size() != 0)
        {
            float objRotatedX = 0.0f;
            float objRotatedY = 0.0f;
            for (int i = 0; i < objListXY.size(); i++)
            {
                ObjectXY objListXY_tmp;
                for (int j = 0; j < objListXY[i].x.size(); j++)
                {
                    Point objPoint(
                        objListXY[i].x[j] - vehPosXY.x,
                        objListXY[i].y[j] - vehPosXY.y);
                    objRotatedX = objPoint.x * std::cos(-vehPosXY.heading) -
                        objPoint.y * std::sin(-vehPosXY.heading);
                    objRotatedY = objPoint.x * std::sin(-vehPosXY.heading) +
                        objPoint.y * std::cos(-vehPosXY.heading);
                    objListXY_tmp.x.push_back(objRotatedX);
                    objListXY_tmp.y.push_back(objRotatedY);
                }
                objRotatedXY.push_back(objListXY_tmp);
            }
            return objRotatedXY;
        }
        else
        {
            return objRotatedXY;
        }
    }

    ObjRectBound AEB::FindObjBound(ObjectSD & objList)
    {
        std::vector<float> vectorS;
        std::vector<float> vectorD;
        ObjRectBound objRectBound;

        for (int i = 1; i < objList.s.size(); i++)
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

    int AEB::FindIndexOfClosestIntersection(
        const float vehPosS,
        std::vector<float> & globalCrossS)
    {
        int indexOfClosestIntersection = 0;
        if (globalCrossS.size() != 0)
        {
            std::vector<float> fakeglobalCrossD(globalCrossS.size(), 0);
            int indexOfClosestIntersection = ClosestWaypoint(
                vehPosS,
                0,
                globalCrossS,
                fakeglobalCrossD,
                0,
                globalCrossS.size());

            if (globalCrossS[indexOfClosestIntersection] - vehPosS <= 0)
            {
                indexOfClosestIntersection = indexOfClosestIntersection + 1;
            }
            return indexOfClosestIntersection;
        }
    }

    void AEB::GetStopLineStopInfo(
        const float stoplineDistance,
        const int closestStopLineId,
        std::vector<LaneType> & laneType)
    {
        if (laneType[2] == LaneType::STOP_LINE &&
            stoplineDistance < STOPLINE_SENSING_ROI)
        {
            if (mStopLineId != closestStopLineId)
            {
                mStopLineId = closestStopLineId;
                mCounterStartTime = ros::Time::now().toSec();
                mVehicleStopTime = COUNTER_RESET_TIME;
            }
            else
            {
                mVehicleStopTime = ros::Time::now().toSec() - mCounterStartTime;
                if (mVehicleStopTime > COUNTER_MAX_TIME)
                {
                    mVehicleStopTime = COUNTER_MAX_TIME;
                }
            }
        }
    }

    bool AEB::IsObjectInStaticStopRegion(
        const float objectX,
        const float objectY,
        const int gearState)
    {
        bool isObjInStaticStopRegion = false;
        if (objectX > MIN_OBJ_DISTANCE &&
            objectX < URGENT_OBJ_X_DISTANCE &&
            objectY > -URGENT_OBJ_Y_DISTANCE &&
            objectY < URGENT_OBJ_Y_DISTANCE &&
            gearState >= 2)
        {
            isObjInStaticStopRegion = true;
#ifdef _AEB_DEBUG_
            std::cout << "/* objectX =  */" << objectX<< '\n';
            std::cout << "/* objectY =  */" << objectY<< '\n';
#endif
        }
        return isObjInStaticStopRegion;
    }

    bool AEB::IsObjectInDynamicStopRegion(
        const float vehPosS,
        const float vehPosD,
        const float carSpeed,
        const float objectS,
        const float objectD)
    {
        bool isObjInDynamicStopRegion = false;
        float frontRoiDistance = FrontRoiDistance(carSpeed);
        if (objectS - vehPosS > MIN_OBJ_DISTANCE &&
            objectS - vehPosS < frontRoiDistance &&
            objectD - vehPosD > -URGENT_OBJ_D_DISTANCE &&
            objectD - vehPosD < URGENT_OBJ_D_DISTANCE)
        {
            isObjInDynamicStopRegion = true;
#ifdef _AEB_DEBUG_
            std::cout << "/* objectS =  */" << objectS<< '\n';
            std::cout << "/* objectD =  */" << objectD<< '\n';
            std::cout << "/* vehPosS =  */" << vehPosS<< '\n';
            std::cout << "/* vehPosD =  */" << vehPosD<< '\n';
#endif
        }
        return isObjInDynamicStopRegion;
    }

    bool AEB::IsObjectsCrossStopRegion(
        const float vehPosS,
        const float vehPosD,
        const float carSpeed,
        const float objSpeedS,
        const float objRecBoundMinS,
        const float objRecBoundRightBound,
        const float objRecBoundLeftBound)
    {
        bool isObjCrossStopRegion = false;
        float frontRoiDistance = FrontRoiDistance(carSpeed);
        if (objRecBoundMinS - vehPosS > 0.0f &&
            objRecBoundMinS - vehPosS < frontRoiDistance &&
            objRecBoundRightBound - vehPosD < -URGENT_OBJ_D_DISTANCE &&
            objRecBoundLeftBound - vehPosD > URGENT_OBJ_D_DISTANCE)
        {
            isObjCrossStopRegion = true;
        }
        return isObjCrossStopRegion;
    }

    int AEB::CountObjectsInStopRegion(
        const float vehPosS,
        const float vehPosD,
        const float carSpeed,
        const int gearState,
        std::vector<ObjectXY> & objRotatedXY,
        std::vector<ObjectSD> & objListSD,
        itri_msgs::BehaviorState & accState,
        const itri_msgs::DetectedObjectArray & objBaseLink,
        const std::vector<ObjectXY> & objListXY,
        const float objZ)
    {
        int objInStopRegion = 0;
        mClosestObjDistance = 100.0f;
        mClosestObjId = -1;
        for (int i = 0; i < objRotatedXY.size(); i++)
        {
            for (int j = 0; j < objRotatedXY[i].x.size(); j++)
            {
                ObjRectBound objRectBound = FindObjBound(objListSD[i]);
                bool isObjInStaticStopRegion = IsObjectInStaticStopRegion(
                    objRotatedXY[i].x[j],
                    objRotatedXY[i].y[j],
                    gearState);
                bool isObjInDynamicStopRegion = IsObjectInDynamicStopRegion(
                    vehPosS,
                    vehPosD,
                    carSpeed,
                    objListSD[i].s[j],
                    objListSD[i].d[j]);
                bool isObjCrossStopRegion = IsObjectsCrossStopRegion(
                    vehPosS,
                    vehPosD,
                    carSpeed,
                    objListSD[i].speedS,
                    objRectBound.minS,
                    objRectBound.rightBound,
                    objRectBound.leftBound);
                if (isObjInStaticStopRegion)
                {
                    objInStopRegion += 1;
                    if (objRotatedXY[i].x[j] < mClosestObjDistance)
                    {
                        mClosestObjDistance = objRotatedXY[i].x[j];
                        mClosestObjId = i;
                    }
                }
                else if (isObjInStaticStopRegion == false &&
                    isObjInDynamicStopRegion == true)
                {
                    float distance = objListSD[i].s[j] - vehPosS;
                    if (distance > 0.0f)
                    {
                        float relativeTTC = CountRelativeTTC(
                            carSpeed,
                            objListSD[i].speedS,
                            distance);
                        if (relativeTTC < MAX_URGENT_RELATIVE_TTC &&
                            relativeTTC > MIN_URGENT_RELATIVE_TTC)
                        {
                            objInStopRegion += 1;
                            if (objListSD[i].s[j] < mClosestObjDistance)
                            {
                                mClosestObjDistance = objListSD[i].s[j];
                                mClosestObjId = i;
                            }
                        }
                    }
                }
                if (mClosestObjId > -1)
                {
                    //Concern object on gui
                    accState.behavior_state = 1;
                    accState.obj_id = (objListSD[mClosestObjId].id);
                    accState.obstacles = true;
                }
            }
            if (mClosestObjId > -1)
            {
              PubRvizObjInAEB(objListSD[mClosestObjId].id, objListXY, objZ);
            }
        }
        return objInStopRegion;
    }

    SpeedCmd AEB::FunctionOutput(
        State behavior,
        VehPosXY & vehPosXY,
        int objInStopRegion,
        const float stoplineDistance,
        double vehicleStopTime,
        std::vector<LaneType> & laneType)
    {
        SpeedCmd output;
        output.acceleration = 0.0f;
        output.state_hint = false;

        // When state is INITIAL or FINISH, do not enable AEB.
        if(behavior == State::INITIAL ||
            behavior == State::FINISH)
        {
            return output;
        }
        else
        {
            if (objInStopRegion > 0)
            {

                output.speed = STOP_SPEED_CMD;
                output.force_speed = true;
                output.acceleration = -100.0f;
                output.state_hint = true;
            }
            else
            {

                    output.state_hint = false;
            }
            return output;
        }
    }

    SpeedCmd AEB::StateFunction(
        State behavior,
        int gearState,
        std::vector<LaneType> & laneType,
        VehPosXY & vehPosXY,
        float vehPosS,
        float vehPosD,
        std::vector<ObjectXY> & objListXY,
        std::vector<ObjectSD> & objListSD,
        std::vector<float> & globalCrossS,
        BehaviorOutput & behaviorOutput,
        itri_msgs::BehaviorState & accState,
        const itri_msgs::DetectedObjectArray & objBaseLink,
        std::map<std::string, SpeedCmd> & accelerationCmd,
        const float objZ)
    {
        SpeedCmd returnAEB;
        float carSpeed = vehPosXY.speed;
        std::vector<ObjectXY> objRotatedXY = ObjRotatedXY(vehPosXY, objListXY);
        int indexOfClosestIntersection = FindIndexOfClosestIntersection(
            vehPosS,
            globalCrossS);
        float stoplineDistance =
            std::abs(globalCrossS[indexOfClosestIntersection] - vehPosS);
        int objInStopRegion = CountObjectsInStopRegion(
            vehPosS,
            vehPosD,
            carSpeed,
            gearState,
            objRotatedXY,
            objListSD,
            accState,
            objBaseLink,
            objListXY,
            objZ);
        GetStopLineStopInfo(
            stoplineDistance,
            indexOfClosestIntersection,
            laneType);
        // Function Output
        returnAEB = FunctionOutput(
            behavior,
            vehPosXY,
            objInStopRegion,
            stoplineDistance,
            mVehicleStopTime,
            laneType);
        if (returnAEB.state_hint)
        {
            std::cout << "AEB is working !!!!!!!!!!!!!!!!!!!!!!" << '\n';
            accelerationCmd["AEB"] = returnAEB;
        }
        return returnAEB;
    }

    void AEB::PubRvizObjInAEB(
        const int objID,
        const std::vector<ObjectXY> & objListXY,
        const float objZ)
    {
        visualization_msgs::Marker rvizObjInAEB;
        rvizObjInAEB.header.frame_id = "/map";
        rvizObjInAEB.header.stamp = ros::Time::now();
        rvizObjInAEB.ns = "points_and_lines";
        rvizObjInAEB.action = visualization_msgs::Marker::ADD;
        rvizObjInAEB.pose.orientation.w = 1.0;
        rvizObjInAEB.id = 23456;
        rvizObjInAEB.type = visualization_msgs::Marker::LINE_STRIP;
        rvizObjInAEB.scale.x = 1.0;
        rvizObjInAEB.color.r = 0.0;
        rvizObjInAEB.color.g = 1.0;
        rvizObjInAEB.color.b = 1.0;
        rvizObjInAEB.color.a = 1.0;
        rvizObjInAEB.lifetime = ros::Duration(0.2);
        int index = -1;

        if (objID >= 0)
        {
            for (int k = 0;k <objListXY.size(); k++)
            {
                if (objListXY[k].id == objID)
                {
                    index = k;
                    break;
                }
            }

            if (index >= 0)
            {
                for (int i = 0;i < objListXY[index].x.size(); i++)
                {
                    geometry_msgs::Point markerPt;
                    markerPt.x = objListXY[index].x[i];
                    markerPt.y = objListXY[index].y[i];
                    if (objListXY[index].z.size() > 0)
                    {
                        markerPt.z = objListXY[index].z[i];
                    }
                    else
                    {
                        markerPt.z = objZ;
                    }
                    rvizObjInAEB.points.push_back(markerPt);
                }
                mPubRvizObjInAEB.publish(rvizObjInAEB);
            }
        }
    }

} //namespace
