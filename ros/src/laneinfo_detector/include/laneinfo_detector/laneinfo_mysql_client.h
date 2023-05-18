#ifndef _LANEINFO_MYSQL_CLIENT_H_
#define _LANEINFO_MYSQL_CLIENT_H_

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <mission_planning/util.h>
#include <mission_planning/mysql_define.h>
#include <itri_msgs/PathSegment.h>
#include <mission_planning/db_query.h>
#include <mission_planning/mysql_client.h>

enum class RoadSignType
{
    ROADSIGN = 0,
    STOPBUMP = 1
};

enum class StopBumpCategory
{
    NONE = 0,
    STOP_LINE,
    LONGER_BUMP,
    SHORTER_BUMP,
    SPEED_DOWN_BUMP
};

class LaneInfoMysqlClient : public MysqlClient
{
private:

    void SwitchStopBumpCategory(StopBumpCategory type, std::string &category);

    RoadSymbol ParseRoadSymbol(std::map<std::string, std::string> &resultMap);

public:

    bool UpdateCarStatus(
        std::string &gpsCoordinate,
        itri_msgs::Waypoint &waypoint);

    std::vector<RoadSymbol> GetStoplineByClane(
        int laneId,
        int startPoint,
        int endPoint);
    std::vector<std::string> QueryStoplineByClane(
        int laneId,
        int startPoint,
        int endPoint);
    std::vector<RoadSymbol> GetStoplineByClaneFromQueryResult(
        std::vector<std::string> &results);

    std::map<std::string, std::string> GetLaneType(
        const int laneId);
    std::vector<std::string> QueryLaneTypeByClane(const int laneId);
    std::map<std::string, std::string> GetLaneTypeByClaneFromQueryResult(
        std::vector<std::string> &results);

    void GetClaneNumberAndOrder(
        int laneId,
        int &number,
        int &order);
    std::vector<std::string> QueryClaneNumber(int laneId);
    void GetClaneNumberFromQueryResult(
        std::vector<std::string> &results,
        int &number,
        int &order);

    std::vector<std::string> QueryRoadSign(
        itri_msgs::PathSegment &info,
        std::string tableName);

    std::vector<RoadSymbol> FindStopBump(itri_msgs::PathSegment &segment);
    void QueryStopbumpByPathInfo(
        itri_msgs::PathSegment &segment,
        std::vector<std::string> &stopBumps);

    std::vector<RoadSymbol> FindRoadSign(itri_msgs::PathSegment &segment);
    void QueryRoadSignByPathInfo(
        itri_msgs::PathSegment &segment,
        std::vector<std::string> &roadSigns);
    std::vector<RoadSymbol> FindRoadSignsFromQueryResult(
        std::vector<std::string> results,
        std::string tableName);

    int GetRoadSignId(int main_id, std::string tableName);
    std::vector<std::string> QueryRoadSignId(
        int main_id,
        std::string tableName);
    int GetRoadSignIdFromQueryResult(std::vector<std::string> results);
};

#endif
