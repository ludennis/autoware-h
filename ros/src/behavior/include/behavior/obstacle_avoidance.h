#ifndef __BEHAVIOR_OBSTACLEAVOIDANCE_H_
#define __BEHAVIOR_OBSTACLEAVOIDANCE_H_

#include "state_machine_node.h"

namespace Behavior
{

    class ObstacleAvoidance : public Core
    {
    public:
        double mObstacleStayTime;
        float mTargetD;
        float mWaitingTime;

    public:
        ObstacleAvoidance();
        PointsSD UnitPathGenerator(const int pointsResolution);
        PointsSD PathGenerator(
            PointsSD unitPath,
            const float vehiclePositionS,
            const float vehiclePositionD,
            const float rollInDistance,
            const float bias,
            const int extraPointTime);
        bool PathEndChecking(
            const float targetD,
            std::vector<float> & wayPointsD);

        std::vector<ObjectSD> SelectCheckingObject(
            const float upperbBound,
            const float lowerBound,
            std::vector<ObjectSD> & objlistSD);
        ObjOnPath ObjectInRange(
            const float safeBoundaryWidth,
            std::vector<ObjectSD> & objListInPathCoordinate);
        ObjOnPath PathWithCollisionChecking(
            std::vector<float> & pathLineS,
            std::vector<float> & pathLineD,
            std::vector<ObjectSD> & objNeedChecking,
            const float safeBoundaryWidth);

        float WaitingTimeGenerator(
            const float vehicleSpeed,
            const float vehiclePositionS,
            const float rollInDistance,
            const float objPositionS);
        int ExtraPointNumber(
            const float rollInDistance,
            const float vehicleSpeed);

        ObjRectBound FindObjBound(ObjectSD objList);

        int FindLaneBias(const float PositionD, const float roadWidth);

        std::vector<bool> ExtraSpaceHint(
            const int laneNum,
            const int laneId);
        std::vector<bool> LaneLineHint(std::vector<LaneType> & laneType);

        float BiasGenerator(
            PointsSD unitPath,
            const float biasResolution,
            const float vehiclePositionS,
            const float vehiclePositionD,
            const float rollInDistance,
            const int searchingStartD,
            const int searchingEndD,
            std::vector<ObjectSD> & frontObjects,
            const int distanceTolerance,
            const int extraPointTime);

        void GenMeterWaypoint(
            const float vehiclePositionS,
            const std::vector<float> & wayPointsS,
            const std::vector<float> & wayPointsD,
            std::vector<float> & wayPointsMeterS,
            std::vector<float> & wayPointsMeterD);

        float FinalPathBiasGenerator(
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
            const int extraPointNum);

        float SelectBias(
            const float leftPathBias,
            const float rightPathBias,
            const float targetD);

        BehaviorOutput SpeedGenerator(
            BehaviorOutput stateVector,
            std::vector<LaneType> & laneType,
            const float vehicleSpeed);

        BehaviorOutput StateFunction(
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
            const int laneId);
    };
}
#endif
