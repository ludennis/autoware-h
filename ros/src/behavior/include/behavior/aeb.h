#ifndef __BEHAVIOR_AEB_H__
#define __BEHAVIOR_AEB_H__

#include <behavior/core.h>
#include <behavior/lane_type.h>
#include <behavior/state.h>
#include <behavior/context.h>

namespace Behavior
{
    class AEB : public Core
    {
    public:
        AEB();
        double mVehicleStopTime;
        double mCounterStartTime;
        int mStopLineId;
        int mClosestObjId;
        float mClosestObjDistance;

        SpeedCmd StateFunction(
            State behavior,
            int gearState,
            std::vector<LaneType> & laneType,
            VehPosXY & vehPosXY,
            const float vehPosS,
            const float vehPosD,
            std::vector<ObjectXY> & objListXY,
            std::vector<ObjectSD> & objListSD,
            std::vector<float> & globalCrossS,
            BehaviorOutput & behaviorOutput,
            itri_msgs::BehaviorState & accState,
            const itri_msgs::DetectedObjectArray & objBaseLink,
            std::map<std::string, SpeedCmd> & accelerationCmd,
            const float objZ);
        float FrontRoiDistance(const float carSpeed);
        float CountRelativeTTC(
            const float carSpeed,
            const float objectSpeed,
            const float distance);
        std::vector<ObjectXY> ObjRotatedXY(
            VehPosXY & vehPosXY,
            std::vector<ObjectXY> & objListXY);
        ObjRectBound FindObjBound(ObjectSD & objList);
        int FindIndexOfClosestIntersection(
            const float vehPosS,
            std::vector<float> & globalCrossS);
        void GetStopLineStopInfo(
            const float stoplineDistance,
            const int closestStopLineId,
            std::vector<LaneType> & laneType);
        bool IsObjectInStaticStopRegion(
            const float objectX,
            const float objectY,
            const int gearState);
        bool IsObjectInDynamicStopRegion(
            const float vehPosS,
            const float vehPosD,
            const float carSpeed,
            const float objectS,
            const float objectD);
        bool IsObjectsCrossStopRegion(
            const float vehPosS,
            const float vehPosD,
            const float carSpeed,
            const float objSpeedS,
            const float objRecBoundMinS,
            const float objRecBoundRightBound,
            const float objRecBoundLeftBound);
        int CountObjectsInStopRegion(
            const float vehPosS,
            const float vehPosD,
            const float carSpeed,
            const int gearState,
            std::vector<ObjectXY> & objRotatedXY,
            std::vector<ObjectSD> & objListSD,
            itri_msgs::BehaviorState & accState,
            const itri_msgs::DetectedObjectArray & objBaseLink,
            const std::vector<ObjectXY> & objListXY,
            const float objZ);
        SpeedCmd FunctionOutput(
            State behavior,
            VehPosXY & vehPosXY,
            int objInStopRegion,
            const float stoplineDistance,
            double vehicleStopTime,
            std::vector<LaneType> & laneType);
        void PubRvizObjInAEB(
            const int objID,
            const std::vector<ObjectXY> & objListXY,
            const float objZ);
    protected:
        ros::NodeHandle mNode;
        ros::Publisher mPubRvizObjInAEB;
    private:
        std::vector<float> mObjpointDistance;
        std::vector<float> mObjpointAngle;
        std::vector<float> mObjpointTtc;
    };
}

#endif // __BEHAVIOR_AEB_H__
