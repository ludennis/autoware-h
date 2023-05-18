#include "node.h"
#include <stdint.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <behavior/lane_type.h>
#include <moving_range_matching.h>

using LaneType = Behavior::LaneType;
using hash_t = size_t;
constexpr hash_t prime = 0x100000001B3ull;
constexpr hash_t basis = 0xCBF29CE484222325ull;

namespace LaneInfoDetector
{
template <typename T>
static inline T GetPrincipalAngle(const T & angle)
{
    return std::fmod(angle, 2.0 * M_PI);
}

hash_t hash_run_time(const char* str) {
    hash_t ret = basis;

    while (*str) {
        ret ^= *str;
        ret *= prime;
        str++;
    }

    return ret;
}

constexpr hash_t hash_compile_time(const char* str, hash_t last_value = basis) {
    return *str ? hash_compile_time(str + 1, (*str ^ last_value) * prime) : last_value;
}

constexpr hash_t operator "" _hash(const char* p, size_t) {
    return hash_compile_time(p);
}

Node::Node()
{
    mPubLaneInfo = mNode.advertise<itri_msgs::lane_info>("lane_info", 1);
    mPubRouteInfo = mNode.advertise<itri_msgs::Route_info>("route_info", 1);
    mPubCrossPath = mNode.advertise<itri_msgs::WaypointArray>("intersection", 1);
    mPubLaneWaypointsInfo =
        mNode.advertise<itri_msgs::lane_waypoints_info>("lane_waypoints_info", 1);

    mSubGlobalPpath = mNode.subscribe("global_path", 1,
        &Node::CallbackGlobalPath, this);
    mSubVehPos = mNode.subscribe("car_state", 1,
        &Node::CallbackVehPos, this);

    mNode.getParam("route", mGlobalPathName);
    std::cout << "mGlobalPathName = " << mGlobalPathName <<'\n';
}

void Node::CallbackGlobalPath(
    const itri_msgs::Path::ConstPtr& msg)
{
    mGlobalPathX.clear();
    mGlobalPathY.clear();
    mGlobalPathHead.clear();
    mGlobalPath.waypoints.clear();
    mGlobalPath.waypoints.assign(msg->waypoints.begin(), msg->waypoints.end());
    for (int i = 0; i < msg->waypoints.size(); i++)
    {
        mGlobalPathX.push_back(
            static_cast<float>(msg->waypoints[i].pose.pose.position.x));
        mGlobalPathY.push_back(
            static_cast<float>(msg->waypoints[i].pose.pose.position.y));
        mGlobalPathHead.push_back(
            static_cast<float>(msg->waypoints[i].pose.pose.orientation.z));
    }
}

void Node::CallbackVehPos(
    const itri_msgs::CarStateConstPtr& msg)
{
    mVehPos.clear();
    mVehPos.push_back(static_cast<float>(msg->pose.pose.position.x));
    mVehPos.push_back(static_cast<float>(msg->pose.pose.position.y));
    mVehPos.push_back(static_cast<float>(msg->pose.pose.orientation.z));
    mVehPos.push_back(static_cast<float>(msg->twist.twist.linear.x));
    mVehPos.push_back(static_cast<float>(msg->twist.twist.angular.z));
}

void UpdateLaneInfo(std::string pathName, int closestWaypoint, int globalPathSize,
    itri_msgs::lane_waypoints_info & laneWaypointsInfo)
{
    //ROUTE INFO DEFAULT =======================================================
    std::pair<int, int> lanetypeDefault =
        {int(LaneType::WHITE_SOLID_LINE), int(LaneType::WHITE_SOLID_LINE)};
    float laneWidthDefault = 3.0;
    unsigned int laneIDDefault = 0;
    unsigned int laneNumDefault = 1;
    unsigned int infoSizeDefault = 100;
    unsigned int infoSize = (globalPathSize-closestWaypoint) <= infoSizeDefault ?
        globalPathSize-closestWaypoint:infoSizeDefault;

    std::vector<int> laneChangeStartIndex;
    std::vector<int> laneChangeEndIndex;
    std::vector<float> laneWidth;
    std::vector<unsigned int> laneID;
    std::vector<unsigned int> laneNum;
    std::vector<std::pair<int, int>> lanetype;
    laneChangeStartIndex.clear();
    laneChangeEndIndex.clear();
    laneWidth.clear();
    laneID.clear();
    laneNum.clear();
    lanetype.clear();
    switch(hash_run_time(pathName.c_str()))
    {
        case "hino-58"_hash:
            laneChangeStartIndex = {23, 799};
            laneChangeEndIndex = {566, 947};
            laneWidth = {laneWidthDefault, laneWidthDefault};
            laneID = {1, 1};
            laneNum = {2, 2};
            lanetype = {{int(LaneType::WHITE_DASHED_LINE),
                int(LaneType::WHITE_SOLID_LINE)}};
                std::cout << "Update Lane Info Hino-58" << '\n';
            break;
        case "hsr"_hash:
            laneChangeStartIndex = {2166, 3890, 6798, 6968,
                7170, 7317, 7903, 9307, 9713, 10248, 14400};
            laneChangeEndIndex = {
                3839, 4321, 6931, 7136, 7276, 7600, 8206, 9670, 10246, 11921, 14848};
            laneWidth = {laneWidthDefault, laneWidthDefault,
                laneWidthDefault, laneWidthDefault, laneWidthDefault, laneWidthDefault,
                laneWidthDefault, laneWidthDefault, laneWidthDefault, laneWidthDefault,
                laneWidthDefault};
            laneID = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
            laneNum = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
            lanetype = {
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)},
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)},
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)},
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)},
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)},
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)},
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)},
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)},
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)},
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)},
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)}};
             std::cout << "Update Lane Info HSR" << '\n';
             break;
        case "operation"_hash:
            laneChangeStartIndex = {23};
            laneChangeEndIndex = {1657};
            laneWidth = {laneWidthDefault};
            laneID = {1};
            laneNum = {2};
            lanetype = {{int(LaneType::WHITE_DASHED_LINE),
                int(LaneType::WHITE_SOLID_LINE)}};
            std::cout << "Update Lane Info Operation" << '\n';
            break;
        case "acc"_hash:
            laneChangeStartIndex = {23, 1485, 1614, 1895};
            laneChangeEndIndex = {1390, 1585, 1656, 2354};
            laneWidth = {laneWidthDefault, laneWidthDefault};
            laneID = {1, 1};
            laneNum = {1, 1};
            lanetype = {{int(LaneType::WHITE_DASHED_LINE),
                int(LaneType::WHITE_SOLID_LINE)}};
            std::cout << "Update Lane Info Operation" << '\n';
        break;
        default: //hino-58
            laneChangeStartIndex = {23, 799};
            laneChangeEndIndex = {566, 947};
            laneWidth = {laneWidthDefault, laneWidthDefault};
            laneID = {1, 1};
            laneNum = {2, 2};
            lanetype = {
             {int(LaneType::WHITE_DASHED_LINE), int(LaneType::WHITE_SOLID_LINE)}};
            break;
    }

    laneWaypointsInfo.point.clear();
    itri_msgs::lane_info laneInfo[infoSize];

    for (int pointIndex = 0;
        pointIndex < infoSize && (pointIndex + closestWaypoint) < (globalPathSize);
        pointIndex++)
    {
        bool infoSave = false;
        for (int i = 0; i < laneChangeStartIndex.size() && infoSave == false; i++)
        {
            if (closestWaypoint + pointIndex  > laneChangeStartIndex[i] &&
                closestWaypoint + pointIndex < laneChangeEndIndex[i])
            {
                laneInfo[pointIndex].lane_width = laneWidth[i];
                laneInfo[pointIndex].lane_id = laneID[i];
                laneInfo[pointIndex].lane_num = laneNum[i];
                laneInfo[pointIndex].line_type.push_back(lanetype[i].first);
                laneInfo[pointIndex].line_type.push_back(lanetype[i].second);
                infoSave = true;
            }
        }
        if (infoSave == false)
        {
            laneInfo[pointIndex].lane_width = laneWidthDefault;
            laneInfo[pointIndex].lane_id = laneIDDefault;
            laneInfo[pointIndex].lane_num = laneNumDefault;
            laneInfo[pointIndex].line_type.push_back(lanetypeDefault.first);
            laneInfo[pointIndex].line_type.push_back(lanetypeDefault.second);
        }
    }

    std::cout << "*****infoSize: " << infoSize << '\n';
    for (int i = 0; i < infoSize; i++)
    {
        laneWaypointsInfo.point.push_back(laneInfo[i]);
    }
}

void SetLaneInfo(std::string pathName, std::vector<int> & bumperIndex,
    std::vector<int> & stopline, std::vector<int> & startTurnRightIndex,
    std::vector<int> & endTurnRightIndex, std::vector<int> & startTurnLeftIndex,
    std::vector<int> & endTurnLeftIndex)
{
    switch(hash_run_time(pathName.c_str()))
    {
        case "hino-58"_hash:
            bumperIndex = {14, 155, 286, 307, 453, 475, 579, 758, 800};
            stopline = {0};
            startTurnRightIndex = {0};
            endTurnRightIndex = {0};
            startTurnLeftIndex = {0};
            endTurnLeftIndex = {0};
            std::cout << "Set LaneInfo Hino-58" << '\n';
            break;
        case "nokia"_hash:
            bumperIndex = {52, 155, 318, 335, 478};
            stopline = {0};
            startTurnRightIndex = {0};
            endTurnRightIndex = {0};
            startTurnLeftIndex = {0};
            endTurnLeftIndex = {0};
            std::cout << "Set LaneInfo Nokia" << '\n';
            break;
        case "hsr"_hash:
            bumperIndex = {14, 97, 135, 233, 8435, 9229,
                13130, 15529, 15566};
            stopline = {278, 530, 714, 899, 1972, 2114, 2724,
                3392, 3844, 4364, 4728, 6179, 6379, 6524, 6613, 6686, 6769,
                6930, 7139, 7277, 7452, 7602, 7874, 8069, 8581, 8712, 8952,
                9217, 9682, 9812, 9914, 11411, 11972, 13105, 13653, 14707,
                14895, 15092, 15252, 15351, 15635};
            startTurnRightIndex = {0};
            endTurnRightIndex = {0};
            startTurnLeftIndex = {0};
            endTurnLeftIndex = {0};
                std::cout << "Set LaneInfo HSR" << '\n';
            break;
        case "operation"_hash:
            bumperIndex = {153, 285, 388, 677, 700, 804, 948, 1071, 1180, 1390, 1666, 1703};
            stopline = {522};
            startTurnRightIndex = {7, 774, 1160, 1484, 1632};
            endTurnRightIndex = {27, 820, 1195, 1529, 1665};
            startTurnLeftIndex = {369, 541, 1080, 1280, 1365, 1572};
            endTurnLeftIndex = {428, 590, 1130, 1328, 1411, 1616};
            std::cout << "Set LaneInfo Operation" << '\n';
            break;
        case "acc"_hash:
            bumperIndex = {0};
            stopline = {0};
            startTurnRightIndex = {0};
            endTurnRightIndex = {0};
            startTurnLeftIndex = {0};
            endTurnLeftIndex = {0};
            std::cout << "Set LaneInfo ACC" << '\n';
            break;
        default: //hino-58
            bumperIndex = {14, 155, 286, 307, 453, 475, 579, 758, 800};
            stopline = {0};
            startTurnRightIndex = {0};
            endTurnRightIndex = {0};
            startTurnLeftIndex = {0};
            endTurnLeftIndex = {0};
            std::cout << "doesn't get the parameter, set the Default: Hino-58" << '\n';
    }
}

void Node::MainLoop()
{
    MovingRangeMatching pathMatch;
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce();

        if (mGlobalPathX.size() > 0 && mVehPos.size() > 0)
        {
            itri_msgs::lane_info mLaneInfoOut;
            itri_msgs::Route_info mRouteInfoOut;
            itri_msgs::Waypoint mPubCrosswayPoints;
            itri_msgs::WaypointArray mPubCrossPathOut;
            itri_msgs::lane_waypoints_info mLaneWaypointsInfoOut;
            mCrossPointsIndex.clear();

            for (int i = 0; i < mGlobalPathX.size(); i++)
            {
                mRouteInfoOut.road_w.push_back(3.0);
                mLaneInfoOut.on_route_hint.push_back(true);
            }

            std::vector<int> bumperIndex;
            std::vector<int> stopline;
            std::vector<int> startTurnRightIndex;
            std::vector<int> endTurnRightIndex;
            std::vector<int> startTurnLeftIndex;
            std::vector<int> endTurnLeftIndex;
            bumperIndex.clear();
            stopline.clear();
            std::string pathName = mGlobalPathName;
            std::cout << "Lane Info pathName = " << pathName <<'\n';

            SetLaneInfo(pathName, bumperIndex, stopline, startTurnRightIndex,
                endTurnRightIndex, startTurnLeftIndex, endTurnLeftIndex);

            for (int i = 0; i < bumperIndex.size(); i++)
            {
                mLaneInfoOut.bumper.push_back(bumperIndex[i]);
            }

            for (int i = 0; i < stopline.size(); i++)
            {
                mCrossPointsIndex.push_back(stopline[i]);
                mLaneInfoOut.traffic_stop_line.push_back(stopline[i]);
            }

            for (int i = 0; i < startTurnRightIndex.size(); i++)
            {
                mLaneInfoOut.start_right_turn.push_back(startTurnRightIndex[i]);
            }

            for (int i = 0; i < endTurnRightIndex.size(); i++)
            {
                mLaneInfoOut.end_right_turn.push_back(endTurnRightIndex[i]);
            }

            for (int i = 0; i < startTurnLeftIndex.size(); i++)
            {
                mLaneInfoOut.start_left_turn.push_back(startTurnLeftIndex[i]);
            }

            for (int i = 0; i < endTurnLeftIndex.size(); i++)
            {
                mLaneInfoOut.end_left_turn.push_back(endTurnLeftIndex[i]);
            }

            for (int i = 0; i < mCrossPointsIndex.size(); i++)
            {
                mPubCrosswayPoints.pose.pose.position.x =
                    mGlobalPathX[mCrossPointsIndex[i]];
                mPubCrosswayPoints.pose.pose.position.y =
                    mGlobalPathY[mCrossPointsIndex[i]];
                mPubCrosswayPoints.pose.pose.orientation.z =
                    mGlobalPathHead[mCrossPointsIndex[i]];
                mPubCrossPathOut.waypoints.push_back(mPubCrosswayPoints);
            }

            mLaneInfoOut.lane_id = 0;
            mLaneInfoOut.lane_num = 1;
            mLaneInfoOut.front_line_dis = 1000.0;
            mLaneInfoOut.line_type.push_back(
                static_cast<uint32_t>(LaneType::EMPTY));
            mLaneInfoOut.line_type.push_back(
                static_cast<uint32_t>(LaneType::EMPTY));
            mLaneInfoOut.line_type.push_back(
                static_cast<uint32_t>(LaneType::EMPTY));

            itri_msgs::CarState carState;
            carState.pose.pose.position.x = mVehPos[0];
            carState.pose.pose.position.y = mVehPos[1];
            carState.pose.pose.orientation.z = mVehPos[2];
            int closestWaypoint =  pathMatch.RunOnce(1.0f, mGlobalPath,
                carState);
            std::cout << "種子種子 closestWaypoint: " << closestWaypoint << '\n';
            //fake_lane_id
            float deltaHeading = GetPrincipalAngle(std::abs(mVehPos[2] -
                mGlobalPathHead[closestWaypoint]));
            if (deltaHeading  > M_PI / 2 &&
                deltaHeading  < 3 * M_PI / 2)
            {
                mLaneInfoOut.lane_id = 0;
            }

            float frontLaneLinePublishDistance =
                std::max(15.0, mVehPos[3] * 3.0);


            //fake_lane_line_type(side)
            //fake_lane_line_type(front)
            if(mLaneInfoOut.lane_id == 0)
            {
                if (mLaneInfoOut.lane_num == 1)
                {
                    mLaneInfoOut.line_type = {
                        static_cast<uint32_t>(LaneType::WHITE_SOLID_LINE),
                        static_cast<uint32_t>(LaneType::WHITE_SOLID_LINE),
                        static_cast<uint32_t>(LaneType::EMPTY)};
                }

                if ((15400 - closestWaypoint < frontLaneLinePublishDistance &&
                    15400 - closestWaypoint > -5) ||
                    (15400 - closestWaypoint < frontLaneLinePublishDistance &&
                    15400 - closestWaypoint > -5) ||
                    (15400 - closestWaypoint < frontLaneLinePublishDistance &&
                    15400 - closestWaypoint > -5) ||
                    (15400 - closestWaypoint < frontLaneLinePublishDistance &&
                    15400 - closestWaypoint > -5))
                {
                    mLaneInfoOut.line_type[2] =
                        static_cast<uint32_t>(LaneType::LONGER_BUMP);
                    mLaneInfoOut.front_line_dis = 1000.0;
                }
            }

            UpdateLaneInfo(pathName, closestWaypoint,
                mGlobalPathX.size(), mLaneWaypointsInfoOut);

            mPubLaneInfo.publish(mLaneInfoOut);
            mPubRouteInfo.publish(mRouteInfoOut);
            mPubCrossPath.publish(mPubCrossPathOut);
            mPubLaneWaypointsInfo.publish(mLaneWaypointsInfoOut);
        }
        loop_rate.sleep();
    }
}

}/* namespace LaneInfoDetector */
