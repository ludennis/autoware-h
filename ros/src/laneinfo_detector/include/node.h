/*
* lanetype_detector.h
*
*  Created on: Apr 17, 2018
*      Author: Jui
*/
#ifndef LANEINFODETECTORNODE_H_
#define LANEINFODETECTORNODE_H_
#include <ros/ros.h>
#include "utils.h"
#include <vector>
#include <string>
#include <itri_msgs/Waypoint.h>
#include <itri_msgs/CarState.h>
#include <itri_msgs/WaypointArray.h>
#include <itri_msgs/lane_info.h>
#include <itri_msgs/lane_waypoints_info.h>
#include <itri_msgs/plan.h>
#include <itri_msgs/Route_info.h>
#include <itri_msgs/Path.h>

namespace LaneInfoDetector
{
    struct LaneInfo
    {
        float laneWidth;
        int laneID;
        int laneNum;
        std::pair<int, int> lanetype;
    };

    class Node
    {
    public:
        Node();
        void MainLoop();
        std::vector<int> mCrossPointsIndex;

    protected:
        void CallbackVehPos(const itri_msgs::CarStateConstPtr&);
        void CallbackGlobalPath(const itri_msgs::Path::ConstPtr&);

    protected:
        ros::NodeHandle mNode;

        //Subscriber
        ros::Subscriber mSubGlobalPpath;
        ros::Subscriber mSubVehPos;
        //Publisher
        ros::Publisher mPubLaneInfo;
        ros::Publisher mPubRouteInfo;
        ros::Publisher mPubCrossPath;
        ros::Publisher mPubLaneWaypointsInfo;
    protected:
        std::string mGlobalPathName;
        std::vector<float> mVehPos;

        std::vector<float> mGlobalPathX;	//topic global_path
        std::vector<float> mGlobalPathY;
        std::vector<float> mGlobalPathHead;
        itri_msgs::WaypointArray mGlobalPath;

        std::vector<float> mGlobalCrossX;
        std::vector<float> mGlobalCrossY;
        std::vector<float> mGlobalCrossHead;
        //Route info
        std::vector<float> mRoadWidth;		//topic route_info
    };
}/* namespace LaneInfoDetector */
#endif /* LANEINFODETECTORNODE_H_ */
