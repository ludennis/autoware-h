#ifndef __BEHAVIOR_STATE_MACHINE_NODE_H__
#define __BEHAVIOR_STATE_MACHINE_NODE_H__

#include <behavior/core.h>
#include <behavior/context.h>
#include <behavior/lane_following.h>
#include <itri_msgs/Waypoint.h>
#include <itri_msgs/CarState.h>
#include <itri_msgs/DetectedObjectArray.h>
#include <itri_msgs/WaypointArray.h>
#include <itri_msgs/lane_info.h>
#include <itri_msgs/lane_waypoints_info.h>
#include <itri_msgs/speed_cmd.h>
#include <itri_msgs/Route_info.h>
#include <itri_msgs/WaypointArrays.h>
#include <itri_msgs/Path.h>
#include <itri_msgs/TrafficLightObjects.h>
#include <itri_msgs/TrafficLightObject.h>
#include <itri_msgs/turn_signal_cmd.h>
#include <itri_msgs/Ars40xObjects.h>
#include <itri_msgs/Ars40xObject.h>
#include <itri_msgs/VehicleState.h>
#include <route_mission_handler/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <vector>
#include <behavior/aeb.h>
#include <behavior/behavior_parallel.h>
#include <std_msgs/Bool.h>

namespace Behavior
{
    class StateMachineNode
    {
    public:
        StateMachineNode();
        State mBehavior;
        std::vector<bool> mOnRouteHint;
        void MainLoop();

        const std::shared_ptr<Context> mContext;
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;

    protected:
        void CallbackVehPos(const itri_msgs::CarStateConstPtr&);
        void CallbackWayPoint(const itri_msgs::WaypointArrayConstPtr&);
        void CallbackObList(const itri_msgs::DetectedObjectArrayConstPtr&);
        void CallbackRadarObjList(const itri_msgs::Ars40xObjectsConstPtr & msg);
        void CallbackRadarAssociObj(const itri_msgs::DetectedObjectArrayConstPtr& msg);
        void CallbackDetectionBoundary(const geometry_msgs::PolygonStamped &);
        void CallbackLaneType(const itri_msgs::lane_infoConstPtr&);
        void CallbackRouteInfo(const itri_msgs::Route_infoConstPtr&);
        void CallbackLaneWaypointsInfo(const itri_msgs::lane_waypoints_infoConstPtr&);
        void CallbackIntersection(const itri_msgs::WaypointArrayConstPtr&);
        void CallbackVehicleOdometry(const itri_msgs::VehicleStateConstPtr&);
        void CallbackTrafficLightObjectsSensor(const itri_msgs::TrafficLightObjects& msg);
        void CallbackTrafficLightObjectsFake(const itri_msgs::TrafficLightObjects& msg);
        void CallbackNokiaAlarm(const std_msgs::Bool & msg);
        void CallbackPlatooning(const itri_msgs::CarStateConstPtr& msg);
        bool CheckTransformListener(tf::StampedTransform & transform, const ros::Time &);
        void CallbackNavigationPath(const route_mission_handler::Path::ConstPtr&);

        void CalculateGlobalPathProperty();
        bool IsReady();
        void UpdateStateMachine();
        void PubRvizObjInAEB(const int objID);
        void PubRvizAccRoi(const Polygon & roi);
        void SetMaxSpeed(std::string pathName, std::string vehicleName,
            float & maxSpeed, const int lastClosestPathPoint);
        void UpdateTurnSignal(const int lastClosestPathPoint,
                const std::vector<std::pair<int, int>> & rightTurn,
                const std::vector<std::pair<int, int>> & lefttTurn,
                itri_msgs::turn_signal_cmd & turnCmd);
        void PubBehavior(
            const std::shared_ptr<BehaviorParallel> & behaviorTree);

        void TrafficLightStatusDecision();
        void LoadIntersectionROIFromFile(const std::string & fileName);
        void LoadIntersectionFOVROIFromFile(const std::string & fileName);
        void LoadROI(const std::string & fileName);
        int AccTarget(
            const std::vector<ObjectXY> & objListXY,
            const std::vector<ObjectSD> & objListSD,
            Polygon accRoiPoly,
            const VehPosSD vehPos,
            bool & obstacleHazard,
            float & shortestDistsnce,
            float & frontVelocity);
        void Visualization();

    protected:
        ros::NodeHandle mNode;
        itri_msgs::plan mBehaviorPlanOut;
        //Subscriber
        ros::Subscriber mSubLocalWayPoints;
        ros::Subscriber mSubVehPos;
        ros::Subscriber mSubObjList;
        ros::Subscriber mSubDetectionBoundary;
        ros::Subscriber mSubLaneInfo;
        ros::Subscriber mSubLanePointsInfo;
        ros::Subscriber mSubRouteInfo;
        ros::Subscriber mSubLaneWaypointsInfo;
        ros::Subscriber mSubIntersection;
        ros::Subscriber mSubVehicleOdometry;
        ros::Subscriber mSubTrafficLightStateSensor;
        ros::Subscriber mSubTrafficLightStateFake;
        ros::Subscriber mSubNokiaAlarm;
        ros::Subscriber mSubPlatooning;
        ros::Subscriber mSubRadar;
        ros::Subscriber mSubRadarAssoci;
        ros::Subscriber mSubNaviPath;
        //Publisher
        ros::Publisher mPubBehaviorState;
        ros::Publisher mPubGlobalPath;
        ros::Publisher mPubGlobalPathMarker;
        ros::Publisher mPubSpeedCmd;
        ros::Publisher mPubTurnCmd;

    protected:
        VehPosXY mVehPosXY;   //topic vehicle_position [x,y,head,speed,yaw rate]
        VehPosXY mFleetVehPosXY;
        VehicleOdometry mVehicleOdometry;
        //waypoint msgs
        std::vector<float> mWayPointsX;     //topic waypoints
        std::vector<float> mWayPointsY;
        std::vector<float> mWayPointsHead;
        std::vector<float> mWayPointsSpeed;
        //Obj
        std::vector<float> mObjWidth;
        //Global cross path
        std::vector<float> mGlobalCrossX;
        std::vector<float> mGlobalCrossY;
        std::vector<float> mGlobalCrossS;
        std::vector<float> mGlobalCrossD;
        std::vector<float> mGlobalCrossHead;
        std::vector<int> mTrafficStopLine;
        std::vector<int> mBumper;
        std::vector<std::pair<int, int>> mRightTurn;
        std::vector<std::pair<int, int>> mLeftTurn;
        std::vector<route_mission_handler::Lane> mNaviLane;

        std::map<int ,std::map<int ,route_mission_handler::Waypoint>> mLaneMap;
        route_mission_handler::Path mNaviPath;

        itri_msgs::WaypointArray mWayPointArry;
        itri_msgs::WaypointArray mAccRoiWaypoints;
        itri_msgs::BehaviorState mBehaviorStateOut;
        pcl::KdTreeFLANN<pcl::PointXYZ> mGlobalPathKdtree;

        bool mCACC;
        bool mCurve;
        bool mParkingSpaceInfoReady;

        double mMaxSpeed;
        double mTimeStep;
        double mTreeTimeStep;
        double mTime;
        double mWayptRes;

        float mLastVehicleSpeed;
        float mTargetAcc;
        float mSpeed;
        float mTargetSpeed;
        float mDeltaDistance;
        float mShortestDistance;

        int mACCconcernedObjId;
        int mBumperCount;
        int mNextBumper;
        int mNextTrafficStopLine;
        int mTrafficLight;
        int mTrafficLightFake;
        int mTrafficLightSensor;
        int mTrafficStopLineCount;

        AEB mAEB;
        std::vector<int> mIndexStartCurve;
        std::vector<int> mIndexEndCurve;
        LaneFollowing mLaneFollowing;
        //Parking information
        tf::TransformListener mTransformListener;
        tf::StampedTransform mTransform;
        //LF info
        std::string mRoute;
        std::string mVehicleName;


    };
}/* namespace Behavior */

#endif /* __BEHAVIOR_STATE_MACHINE_NODE_H__ */
