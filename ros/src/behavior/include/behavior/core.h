#ifndef __BEHAVIOR_CORE_H__
#define __BEHAVIOR_CORE_H__

#include <itri_msgs/DetectedObject.h>
#include <itri_msgs/DetectedObject.h>
#include <itri_msgs/DetectedObjectArray.h>
#include <itri_msgs/Ars40xObjects.h>
#include <itri_msgs/Ars40xObject.h>
#include <behavior/types.h>
#include <behavior/state.h>
#include <itri_msgs/plan.h>
#include <itri_msgs/BehaviorState.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_listener.h>
#include <vector>
#include <behavior/state.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>
#include <itri_msgs/WaypointArray.h>
#include <itri_msgs/WaypointArrays.h>

namespace Behavior
{
    typedef boost::geometry::model::point<
        float, 2, boost::geometry::cs::cartesian> BoostPoint;
    typedef boost::geometry::model::polygon<BoostPoint> BoostPolygon;

    struct Polygon
    {
        BoostPolygon corners;
        float id;
    };

    class Core
    {
    public:
        void ToSpeedXY(
            const VehPosXY & vehPosXY,
            ObjectXY & objectXY,
            const float steeringAngle);
        SpeedCmd SetSpeedCmd(
            const bool force_speed,
            const float targetSpeed,
            const float acceleration);
        void LocalPathROI(
            const itri_msgs::WaypointArray & LocalWayPointsXY,
            const std::vector<float> & LocalX,
            const std::vector<float> & LocalY,
            const VehPosXY & vehPosXY,
            itri_msgs::WaypointArray & AccRoiXY);
        void ToSpeedSD(
            const ObjectXY & objectXY,
            ObjectSD & objListSD,
            const std::vector<float> & globalX,
            const std::vector<float> & globalY,
            const std::vector<float> & globalHeading,
            const int lastClosestWaypoint,
            const int searchingRange,
            const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &);
        std::vector<float> GetFrenet(
            float localX,
            float localY,
            float Theta,
            const std::vector<float> & globalX,
            const std::vector<float> & globalY,
            int lastClosestWaypoint,
            int searchingRange);
        std::vector<float> GetFrenet(
            float localX,
            float localY,
            float Theta,
            const std::vector<float> & globalX,
            const std::vector<float> & globalY,
            int lastClosestWaypoint,
            int searchingRange,
            const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &);
        void SetBehaviorOutput(
            itri_msgs::plan & behaviorPlanOut,
            BehaviorOutput & behaviorOutput);
        void DecisionState(
            const std::string tagState, itri_msgs::BehaviorState & behaviorState,
            const int trafficState, const float OAGenBiasObj, const float OASlowDownObj,
            const float IAObj, const float ACCObj, const bool OAGenBias);
        void BehaviorStateOutput(
            const State & behavior,
            const bool obstacleHazard,
            itri_msgs::BehaviorState & behaviorState);
        void BumperDetect(
            bool & bumper, int & currentBumper, const int nextBumper,
            const int lastClosestWaypoint, const float maxSpeed, const float vehicleS);
        void CurveDetect(
            bool & enterCurve, const float vehiclePositionS,
            const std::vector<float> & globalPathS,
            const std::vector<float> & globalPathCurvature,
            const std::vector<int> & indexStartCurve,
            const std::vector<int> & indexEndCurve,
            const float maxSpeed,
            float & curveSpeed);
        void TransformObjectXY(
            const tf::StampedTransform & transform,
            ObjectXY & objPoints,
            const itri_msgs::DetectedObject& object);
        void TransformObjectXY(
            const tf::StampedTransform & transform,
            ObjectXY & objPoints,
            const itri_msgs::Ars40xObject& object);
        std::vector<float> LaneOccupied(
            float vehPosD,
            float roadWidth,
            float rightBound,
            float leftBound);
        ObjRectBound ObjBound(const ObjectSD & objList);
        std::vector<bool> LaneSafeHint(
            float roadWidth,
            std::vector<float> laneOccupiedspace,
            ObjectHints objectHints);
        float ObjDistance(
            float vehPosS,
            float objFront,
            float objBack,
            float frontSafeDist,
            float backSafeDist);
        ObjectHints ObjectInLaneDetector(
            int dist2Map,
            float vehPosS,
            float vehPosD,
            float vehSpeed,
            const float rollInFactor,
            const std::vector<float> & roadWidth,
            const std::vector<ObjectSD> & objList);
        int ClosestWaypoint(
            float nowX,
            float nowY,
            const std::vector<float> & GlobalX,
            const std::vector<float> & GlobalY,
            int lastClosestWaypoint,
            int searchingRange,
            const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &);
        int ClosestWaypoint(
            float nowX,
            float nowY,
            const std::vector<float> & GlobalX,
            const std::vector<float> & GlobalY,
            int lastClosestWaypoint,
            int searchingRange);
        int NextWaypoint(
            float nowX,
            float nowY,
            float Theta,
            const std::vector<float> & GlobalX,
            const std::vector<float> & GlobalY,
            int lastClosestWaypoint,
            int searchingRange);
        int NextWaypoint(
            float nowX,
            float nowY,
            float Theta,
            const std::vector<float> & GlobalX,
            const std::vector<float> & GlobalY,
            int lastClosestWaypoint,
            int searchingRange,
            const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &);
        void FrenetTransform(
            float localX,
            float localY,
            float Theta,
            const std::vector<float> & globalX,
            const std::vector<float> & globalY,
            float & globalS,
            float & globalD,
            int lastClosestWaypoint,
            int searchingRange,
            const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &);
        void FrenetTransform(
            std::vector<float> & localX,
            std::vector<float> & localY,
            std::vector<float> & Theta,
            const std::vector<float> & globalX,
            const std::vector<float> & globalY,
            std::vector<float> & globalS,
            std::vector<float> & globalD,
            int lastClosestWaypoint,
            int searchingRange,
            const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &);
        void FrenetTransform(
            std::vector<ObjectXY> & objListXY,
            const std::vector<float> & globalX,
            const std::vector<float> & globalY,
            const std::vector<float> & globalHeading,
            const VehPosXY & vehPosXY,
            std::vector<ObjectSD> & objListSD,
            int lastClosestWaypoint,
            int searchingRange,
            const float steeringAngle,
            const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &);
        float GetRadOfCurvature(
            const float & prevX, const float & prevY,
            const float & targetX, const float & targetY,
            const float & nextX, const float & nextY);
        float MedianFilter(std::vector<float> & input, const int window);
        void GetMedianFilterVector(
            const std::vector<float> & input,
            std::vector<float> & output,
            const int window);
        void SetSpeedProfile(
            const std::map<std::string, SpeedCmd> & accelerationCmd,
            float & speed, float & targetAcc, float & targetSpeed,
            const double durationTime, std::string & tagState, State & behavior);
        void SpeedProtection(
            float & speed, float & targetAcc,float & targetSpeed,
            const double durationTime, const float CmdSpeed, const float CmdAcce);
        void OrderClockwise(Polygon & poly);
        void AddPoints(Polygon & pool, const BoostPoint & newpoint);
        Polygon ObjListXYToPoly(const ObjectXY & object);
        float PolygonArea(const Polygon & poly);
        float IntersectionArea(Polygon & poly1, Polygon & poly2);
        bool isPointInPolygon(PointXY & point, Polygon & polygon);
        Polygon LaneToPoly(
            const itri_msgs::WaypointArray & lane, const float roadWidth);
        void CalculatePathCurvature(
            const itri_msgs::WaypointArray & path,
            std::vector<float> & curvature);
        void PathOffset(
            const itri_msgs::WaypointArray & Oldpath,
            const float bias,
            const std::vector<float> curvature,
            std::vector<PointXY> & newPath);
        bool IsCurveUnderThreshold(
            const float curvature, const float offset);
        void PointOffset(
            const float distance, itri_msgs::Waypoint & point);
    };
}/* namespace Behavior */

#endif /* __BEHAVIOR_CORE_H__ */
