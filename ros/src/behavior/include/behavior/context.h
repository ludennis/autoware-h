#ifndef __BEHAVIOR_CONTEXT_H__
#define __BEHAVIOR_CONTEXT_H__

#include <behavior/core.h>
#include <behavior/lane_type.h>
#include <debug_to_file/debug_to_file.h>
#include <itri_msgs/Waypoint.h>
#include <itri_msgs/CarState.h>
#include <itri_msgs/WaypointArray.h>
#include <itri_msgs/lane_info.h>
#include <itri_msgs/Route_info.h>
#include <itri_msgs/WaypointArrays.h>
#include <itri_msgs/Path.h>
#include <itri_msgs/turn_signal_cmd.h>
#include <itri_msgs/VehicleState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <vector>
#include <map>
#include <mutex>
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <route_mission_handler/Path.h>


namespace Behavior
{
    struct Context
    {
    public:
        itri_msgs::BehaviorState BTStateOut;
        SpeedCmd aebCommand;
        std::vector<LaneType> laneType;

        VehPosSD vehPosSD;
        VehPosXY vehPosXY;
        VehPosXY fleetVehPosXY;
        //waypoint msgs
        std::vector<float> wayPointsS;
        std::vector<float> wayPointsD;
        std::vector<float> relativeWayPointsX;
        std::vector<float> relativeWayPointsY;
        std::vector<float> relativeWayPointsHeading;
        std::vector<float> mWayPointsHead;
        std::vector<float> wayPointsX;
        std::vector<float> wayPointsY;
        float relativeOriginX;
        float relativeOriginY;

        //Obj
        std::vector<ObjectSD> objListSD;
        std::vector<ObjectXY> objListXY;
        std::vector<float> objListX;
        std::vector<float> objListY;

        std::vector<ObjectSD> objRadarSD;
        std::vector<ObjectXY> objRadarXY;

        std::map<int, ObjectXY> objLiRarXY;
        std::vector<ObjectXY> objLiRarXYACC;
        std::vector<ObjectSD> objLiRarSD;

        float objRelativeSpeedX;
        float objRelativeSpeedY;
        float objRelativeDistance;
        float objX;
        float objY;
        itri_msgs::DetectedObjectArray mObjBaseLink;

        //Global path
        std::vector<float> globalPathS;
        std::vector<float> globalPathD;
        std::vector<float> globalPathX;
        std::vector<float> globalPathY;
        std::vector<float> globalPathZ;
        std::vector<float> globalPathHead;
        std::vector<float> globalPathCurvature;
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr globalPathKdtree;

        //Lane info
        int laneId;
        int laneNum;
        int currentBumper;
        int currentStopLine;
        bool CACC;
        bool OAGenBias;
        bool bumper;
        bool enterCurve;
        bool obstacleHazard;
        //Route info
        std::vector<float> roadWidth;
        std::vector<LaneInfo> laneData;

        double paramRollInFactor;
        double timeStep;
        int lastClosestPathPoint;
        int searchingRange;
        std::mutex mutex;

        //ConcernedObj info
        int OAmMaxBiasId;
        int OAmSlowDownObjId;
        ObjectXY IAobj;
        ObjectXY ACCObj;
        std::vector<ObjectXY> ACCFrontObjs;

        //Intersection info
        std::vector<int> ROIIds;
        std::vector<int> ROIDirections;
        float accAccelerate;
        float relativeSpeedXUpperBound;
        float relativeSpeedXLowerBound;
        float disToCollision;
        float TTC;
        float OTTC;
        float EDTC;
        float accSpeed;
        float curveSpeed;
        float maxSpeed;
        float speed;
        float minSuggestedCmd;
        std::vector<geometry_msgs::Point> velocitySection;
        int collisionPointIndex;
        std::vector<geometry_msgs::Point> pathToIABlocker;

        std::vector<route_mission_handler::Lane> naviLane;
        std::map<int ,route_mission_handler::Lane> laneInfoMap;
        std::map<int ,std::map<int ,route_mission_handler::Waypoint>> laneMap;
        std::vector<route_mission_handler::Waypoint> globalPath;
        route_mission_handler::Path naviPath;

        //IntersectionFOV info
        std::vector<int> FOVROIIds;
        std::vector<int> FOVROIDirections;



        //start lane bias
        bool LaneChange;

        ros::Publisher pubBehaviorPlan;
        Json::Value intersectionROIs;
        Json::Value intersectionFOVROIs;

        //time
        float objVelocityYTime;

        int trafficLightFake;
        int trafficLight;

        bool avoidanceNeedDistance;

        //switch parking
        bool switchParking;

        //gear state
        int gearState;

        std::shared_ptr<DebugToFile> debug;

        geometry_msgs::Polygon detectionBoundary;
        VehicleOdometry vehicleOdometry;

        std::map<std::string, SpeedCmd> accelerationCmd;
        std::string tagState;
        std::map<std::string, visualization_msgs::MarkerArray> actionMarkers;
        visualization_msgs::MarkerArray allMarkers;
        itri_msgs::turn_signal_cmd turnSignalCmd;

        ros::Publisher pubActionMarkers;
    };
}/* namespace Behavior */

#endif /* __BEHAVIOR_CONTEXT_H__ */
