#ifndef _LANEINFO_OBTAIN_NODE_H_
#define _LANEINFO_OBTAIN_NODE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <itri_msgs/WaypointArray.h>
#include <itri_msgs/Route_info.h>
#include <itri_msgs/lane_info.h>
#include <itri_msgs/CarState.h>
#include <itri_msgs/Path.h>
#include <itri_msgs/CarState.h>
#include <itri_msgs/StationInfo.h>
#include <itri_msgs/StationInfoArray.h>
#include <itri_msgs/LedWrite.h>
#include <mission_planning/convert_gps_coordinate.h>
#include <mission_planning/mysql_client.h>
#include <sensor_msgs/NavSatFix.h>
#include <laneinfo_detector/laneinfo_mysql_client.h>
#include <laneinfo_detector/broadcast_util.h>
#include <algorithm>
#include <deque>
#include <iostream>
#include <exception>

class LaneInfoObtain
{
private:

    bool mIsParkingPathSegment = false;
    bool mIsArrived = false;
    int mWaypointIndex;
    int mClosetRoadSignType;
    int mCurrentLaneId;
    int mStationIndex;
    int mCurrentTurnType;
    int mSearchIndex;
    int mGlobalPathWaypoints;
    float mVehicleSpeed;

    std::vector<itri_msgs::Waypoint> mWaypoints;
    std::vector<float> mSumDistanceOfWaypoints;
    std::vector<int> mDirectionOfWaypoints;
    std::vector<itri_msgs::StationInfo> mStations;

    std::map<int, itri_msgs::PathSegment> mPathSegments;
    std::map<int, std::vector<int>> mLaneNumberOrders;
    std::map<int, std::vector<int>> mLaneLineTypes;
    std::map<int, RoadSymbol> mRoadSignMap;
    std::map<int, itri_msgs::Waypoint> mRoadSignWaypointMap;

    itri_msgs::Waypoint mVehicleWaypoint;
    itri_msgs::lane_info mLaneInfo;
    itri_msgs::WaypointArray mStoplineArray;

    RoadSymbol mClosestRoadSign;
    itri_msgs::Waypoint mClosestRoadSignWaypoint;

    Point3d mVehicleGPS;
    ConvertGPSCoordinate mCoordinateConverter;
    LaneInfoMysqlClient mMysqlClient;

    ros::Publisher mDynamicRoadSignPub; // publish topic "lane_info"
    ros::Publisher mRouteInfoPub; // publish topic "route_info"
    ros::Publisher mStoplineArrayPub; // publish topic "intersection"
    ros::Publisher mArrivalDestionPub; // publish topic "intersection"
    ros::Publisher mLEDMessagePub; // publish topic "intersection"

    bool CheckArrival(int totalWaypointNumber, int currentWaypointIndex);

    void Reset();

    void UpdateVehicleLocationToDatabase(
        Point3d &location,
        itri_msgs::Waypoint &point);

    void CheckWaypointData(
        std::vector<itri_msgs::Waypoint> &waypoints,
        std::vector<itri_msgs::PathSegment> segments);

public:
    LaneInfoObtain();

    LaneInfoObtain(ros::NodeHandle& nh);

    void CarStatusCallback(const itri_msgs::CarStateConstPtr& msg);

    void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg);

    void PathSegmentCallback(const itri_msgs::Path::ConstPtr &msg);

    void StationInfoCallback(
        const itri_msgs::StationInfoArray::ConstPtr &msg);

    void GetNumberOrderOfLane(
        int laneId,
        std::map<int, std::vector<int>> &laneNumberOrders);

    int FindRoadSymbolIndex(
        std::deque<RoadSymbol> symbols,
        itri_msgs::Waypoint &point);

    std::deque<RoadSymbol> GetRoadSignsOfLane(itri_msgs::PathSegment &segment);

    std::deque<RoadSymbol> GetStopBumpsOfLane(itri_msgs::PathSegment &segment);

    void GetStoplinesOfLane(
        itri_msgs::PathSegment &segment,
        std::vector<itri_msgs::Waypoint> &stoplines);

    bool AddToRoadSignsMap(
        std::deque<RoadSymbol> &symbols,
        itri_msgs::Waypoint &point,
        int index);

    void GetRoadSignsMap(
        std::deque<RoadSymbol> &stopBumps,
        std::deque<RoadSymbol> &roadSigns,
        itri_msgs::Waypoint &point,
        int index);

    void GetRoadSigns(
        int &currentIndex,
        RoadSymbol &closetRoadSign,
        std::vector<itri_msgs::Waypoint> &waypoints);

    void GetLineTypeOfLane(
        int &laneId,
        std::map<int, std::vector<int>> &lineTypes);

    void GetRoadLineType(int &laneId, int &currentLaneId);

    void GetLaneInfo(int newLaneId, int &currentLaneId);

    void GetRemindTimeDistance(int currentIndex, std::vector<float> &distances);

    void GetDirection(
        int currentIndex,
        std::vector<int> &directions,
        int searchIndex);

    std::string GetStations(
        int currentIndex,
        std::vector<itri_msgs::StationInfo> &stations);

    void PublishLEDDisplay(std::string texts);

    void GetPathInformation(
        std::vector<itri_msgs::Waypoint> &points,
        std::map<int, itri_msgs::PathSegment> &pathSegments);

    void GetStoplines(
        itri_msgs::PathSegment &segment,
        std::vector<RoadSymbol> &stoplines);

};

#endif
