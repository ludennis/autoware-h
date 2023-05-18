#include <time.h>
#include <route_mission_handler/route_data_handler.h>

LaneConstructor::LaneConstructor(ros::NodeHandle& node,
    ros::NodeHandle& privateNode):
    mLanes(),
    mLanePoints(),
    mNavgRoadMap(),
    mLaneNavgMap(),
    mLaneMarkers(),
    mRoadMarkerMap(),
    mRoadLineMap(),
    mTrafficLightMap()
{
    std::string navgcroads;
    std::string lanes;
    privateNode.param<std::string>("route", mRoute, "");
    privateNode.param<std::string>("navgcroad", navgcroads, "");
    privateNode.param<std::string>("lanes", lanes, "");
    ROS_INFO("route_data_handler init, mRoute %s, navgcroads %s",
        mRoute.c_str(), navgcroads.c_str());

    mRouteNavgcroads = ParseKeyValues(
        navgcroads,
        ',',
        ':',
        mRouteNavgcroadOrders);
    mDebugLanes = ParseStringToIntArray(lanes, ',');
    ROS_INFO("route_data_handler mRouteNavgcroads %ld, mRouteNavgcroadOrders %ld",
        mRouteNavgcroads.size(), mRouteNavgcroadOrders.size());

    mPubFinal = node.advertise<route_mission_handler::Path>
        ("/navigation_path", 1);
    mDisplayMgr = new DisplayManager(node);
}

void LaneConstructor::WaypointsCallback(
    const route_mission_handler::PathsArray::ConstPtr &msg)
{
    mPointsMap.clear();
    mLanePoints.clear();
    geometry_msgs::Point prePnt;
    ROS_INFO("WaypointsCallback msg paths size %ld", msg->paths.size());
    for (int i=0 ; i < msg->paths.size(); i++)
    {
        route_mission_handler::Paths path = msg->paths.at(i);
        for (int j = 0; j < path.lanes.size(); j++)
        {
            int pointId = 1;
            route_mission_handler::Lanes lane = path.lanes.at(j);
            prePnt.x = 0.0;
            prePnt.y = 0.0;
            prePnt.z = 0.0;

            if (!mLanes.count(lane.id))
            {
                mLanes.insert(std::pair<int, route_mission_handler::Lanes>(
                    lane.id,
                    lane));
            }
            if (!mLanePoints.count(lane.id))
            {
                mLanePoints.insert(std::pair<int, std::vector<std::string>>(
                    lane.id,
                    std::vector<std::string>()));
            }

            bool isTurn = mLanes.at(lane.id).isTurn;
            for (int k =0 ; k < lane.points.size(); k++)
            {
                route_mission_handler::Waypoints waypnt = lane.points.at(k);
                if (abs(waypnt.pose.position.x) > 1000000 &&
                    abs(waypnt.pose.position.y) > 1000000)
                {
                    geometry_msgs::Point tempPnt = TransformLocalCartesian(
                        waypnt.pose.position.x,
                        waypnt.pose.position.y,
                        waypnt.pose.position.z);
                    waypnt.pose.position = tempPnt;
                }

                std::string key = std::to_string(waypnt.laneId) + "_";
                if (pointId < twoDigitalThreshold)
                {
                    key += "00";
                }
                else if (pointId < threeDigitalThreshold)
                {
                    key += "0";
                }
                key += std::to_string(pointId);

                if (isTurn &&
                    k < lane.points.size() - 1 &&
                    mLanePoints.at(lane.id).size() > 0)
                {
                    std::string lastKey = mLanePoints.at(lane.id).at(
                        mLanePoints.at(lane.id).size() - 1);
                    if (mPointsMap.count(lastKey))
                    {
                        float dist = GetDistanceBetweenDimensionPoints(
                            mPointsMap.at(lastKey).pose.position,
                            waypnt.pose.position);
#ifdef DEBUG
                        ROS_INFO("WaypointsCallback, waypnt %d(%d) %f,%f,%f, "
                            "lastKey %s (%f,%f,%f), dist %f",
                            waypnt.laneId, waypnt.pointId,
                            waypnt.pose.position.x,
                            waypnt.pose.position.y,
                            waypnt.pose.position.z,
                            lastKey.c_str(),
                            mPointsMap.at(lastKey).pose.position.x,
                            mPointsMap.at(lastKey).pose.position.y,
                            mPointsMap.at(lastKey).pose.position.z,
                            dist);
#endif
                        if (dist < 0.9)
                        {
#ifdef DEBUG
                            ROS_WARN("WaypointsCallback, lastKey %s, dist %f, "
                                "too small !!!",
                                lastKey.c_str(), dist);
#endif
                            continue;
                        }
                    }
                }
                prePnt.x = waypnt.pose.position.x;
                prePnt.y = waypnt.pose.position.y;
                prePnt.z = waypnt.pose.position.z;
                mLanes.at(lane.id).points.push_back(waypnt);

                mLanePoints.at(lane.id).push_back(key);
                mPointsMap.insert(
                    std::pair<std::string, route_mission_handler::Waypoints>(
                        key, waypnt));
                pointId ++;
            }
        }
        ROS_INFO("WaypointsCallback path %s, path.lanes size %ld, "
            "mPointsMap size %ld, mLanePoints size %ld, mLanes size %ld",
            path.name.c_str(), path.lanes.size(),
            mPointsMap.size(), mLanePoints.size(), mLanes.size());
    }
}

void LaneConstructor::NavgRoadsCallback(
    const route_mission_handler::NavgRoadArray::ConstPtr &msg)
{
    ROS_INFO("NavgRoadsCallback msg paths size %ld", msg->navgroads.size());
    for (int j=0 ; j < msg->navgroads.size(); j++)
    {
        route_mission_handler::NavgRoad navgroad = msg->navgroads.at(j);
        ROS_INFO("NavgRoadsCallback navgroad %d, point size %ld, "
            "positiveLaneIds size %ld, negativeLaneIds size %ld",
            navgroad.id, navgroad.points.size(),
            navgroad.positiveLaneIds.size(),
            navgroad.negativeLaneIds.size());
        if (!mNavgRoadMap.count(navgroad.id))
        {
            mNavgRoadMap.insert(std::pair<int, route_mission_handler::NavgRoad>(
                navgroad.id,
                navgroad));
        }
        if (mNavgRoadMap.at(navgroad.id).points.size() == 0)
        {
            mNavgRoadMap.at(navgroad.id).points = navgroad.points;
            for (int i=0; i < mNavgRoadMap.at(navgroad.id).points.size(); i++)
            {
                if (abs(mNavgRoadMap.at(navgroad.id).points.at(i).pose.position.x) > 100000 &&
                    abs(mNavgRoadMap.at(navgroad.id).points.at(i).pose.position.y) > 100000)
                {
                    geometry_msgs::Point tempPnt = TransformLocalCartesian(
                        mNavgRoadMap.at(navgroad.id).points.at(i).pose.position.x,
                        mNavgRoadMap.at(navgroad.id).points.at(i).pose.position.y,
                        mNavgRoadMap.at(navgroad.id).points.at(i).pose.position.z);
                    mNavgRoadMap.at(navgroad.id).points.at(i).pose.position = tempPnt;
                }
            }
        }
    }
    ROS_INFO("NavgRoadsCallback done, mNavgRoadMap size %ld",
        mNavgRoadMap.size());
    if (mLaneNavgMap.size() > 0)
    {
        for (auto it: mLaneNavgMap)
        {
            UpdateNavgRoadLanes(mNavgRoadMap, it.second);
        }
    }
}

void LaneConstructor::UpdateNavgRoadLanes(
    std::map<int, route_mission_handler::NavgRoad> &navgRoads,
    LaneNavg &laneNavg)
{
    if (navgRoads.count(laneNavg.navgroad1_id))
    {
        if (laneNavg.isPositive1 && std::find(
            navgRoads.at(laneNavg.navgroad1_id).positiveLaneIds.begin(),
            navgRoads.at(laneNavg.navgroad1_id).positiveLaneIds.end(),
            laneNavg.laneId) ==
                navgRoads.at(laneNavg.navgroad1_id).positiveLaneIds.end())
        {
            navgRoads.at(laneNavg.navgroad1_id).positiveLaneIds.push_back(
                laneNavg.laneId);
            navgRoads.at(laneNavg.navgroad1_id).positiveLaneNum =
                navgRoads.at(laneNavg.navgroad1_id).positiveLaneIds.size();
            ROS_INFO("UpdateNavgRoadLanes  navgRoads %d update positiveLaneIds "
                "size %ld",
                laneNavg.navgroad1_id,
                navgRoads.at(laneNavg.navgroad1_id).positiveLaneIds.size());
        }
        else if (!laneNavg.isPositive1 && std::find(
            navgRoads.at(laneNavg.navgroad1_id).negativeLaneIds.begin(),
            navgRoads.at(laneNavg.navgroad1_id).negativeLaneIds.end(),
            laneNavg.laneId) ==
                navgRoads.at(laneNavg.navgroad1_id).negativeLaneIds.end())
        {
            navgRoads.at(laneNavg.navgroad1_id).negativeLaneIds.push_back(
                laneNavg.laneId);
            navgRoads.at(laneNavg.navgroad1_id).negativeLaneNum =
                navgRoads.at(laneNavg.navgroad1_id).negativeLaneIds.size();
            ROS_INFO("UpdateNavgRoadLanes  navgRoads %d update negativeLaneIds "
                "size %ld",
                laneNavg.navgroad1_id,
                navgRoads.at(laneNavg.navgroad1_id).negativeLaneIds.size());
        }
    }

    if (navgRoads.count(laneNavg.navgroad2_id))
    {
        if (laneNavg.isPositive2 && std::find(
            navgRoads.at(laneNavg.navgroad2_id).positiveLaneIds.begin(),
            navgRoads.at(laneNavg.navgroad2_id).positiveLaneIds.end(),
            laneNavg.laneId) ==
                navgRoads.at(laneNavg.navgroad2_id).positiveLaneIds.end())
        {
            navgRoads.at(laneNavg.navgroad2_id).positiveLaneIds.push_back(
                laneNavg.laneId);
            navgRoads.at(laneNavg.navgroad2_id).positiveLaneNum =
                navgRoads.at(laneNavg.navgroad2_id).positiveLaneIds.size();
            ROS_INFO("UpdateNavgRoadLanes  navgRoads %d update positiveLaneIds "
                "size %ld",
                laneNavg.navgroad2_id,
                navgRoads.at(laneNavg.navgroad2_id).positiveLaneIds.size());
        }
        else if (!laneNavg.isPositive2 && std::find(
            navgRoads.at(laneNavg.navgroad2_id).negativeLaneIds.begin(),
            navgRoads.at(laneNavg.navgroad2_id).negativeLaneIds.end(),
            laneNavg.laneId) ==
                navgRoads.at(laneNavg.navgroad2_id).negativeLaneIds.end())
        {
            navgRoads.at(laneNavg.navgroad2_id).negativeLaneIds.push_back(
                laneNavg.laneId);
            navgRoads.at(laneNavg.navgroad2_id).negativeLaneNum =
                navgRoads.at(laneNavg.navgroad2_id).negativeLaneIds.size();
            ROS_INFO("UpdateNavgRoadLanes  navgRoads %d update negativeLaneIds "
                "size %ld",
                laneNavg.navgroad2_id,
                navgRoads.at(laneNavg.navgroad2_id).negativeLaneIds.size());
        }
    }
}

void LaneConstructor::LaneNavgRoadsCallback(
    const route_mission_handler::LaneNavgRoadArray::ConstPtr &msg)
{
    ROS_INFO("LaneNavgRoadsCallback LaneNavgRoadArray msg size %ld",
        msg->lanenavgroads.size());
    for (int i=0 ; i< msg->lanenavgroads.size(); i++)
    {
        route_mission_handler::LaneNavgRoad lanenavgroad = msg->lanenavgroads.at(i);
        LaneNavg newLaneNavg;
        newLaneNavg.laneId = lanenavgroad.lane_id;
        newLaneNavg.navgroad1_id = lanenavgroad.navgroad1;
        newLaneNavg.navgroad2_id = lanenavgroad.navgroad2;
        newLaneNavg.isPositive1 = lanenavgroad.isPositive1;
        newLaneNavg.isPositive2 = lanenavgroad.isPositive2;
        newLaneNavg.laneOrder = lanenavgroad.laneno;
        newLaneNavg.seqner = lanenavgroad.seqner;

        // update
        if (mNavgRoadMap.size() > 0)
        {
            UpdateNavgRoadLanes(mNavgRoadMap, newLaneNavg);
        }
        mLaneNavgMap.insert(std::pair<int, LaneNavg>(
            lanenavgroad.lane_id,
            newLaneNavg));
    }
    ROS_INFO("LaneNavgRoadsCallback, mLaneNavgMap size %ld",
        mLaneNavgMap.size());
}

void LaneConstructor::RoadLinesCallback(
    const route_mission_handler::RoadLineArray::ConstPtr &msg)
{
    mRoadLineMap.clear();
    ROS_INFO("RoadLinesCallback msg lines size %ld", msg->lines.size());
    for (int i=0 ; i < msg->lines.size(); i++)
    {
        route_mission_handler::RoadLine obj = msg->lines.at(i);
        if (obj.points.size() > 0 &&
            abs(obj.points.at(0).position.x) > 10000)
        {
            for (int j =0 ; j < obj.points.size(); j++)
            {
                route_mission_handler::RoadLinePoint linePnt = obj.points.at(j);
                geometry_msgs::Point tempPnt = TransformLocalCartesian(
                    linePnt.position.x,
                    linePnt.position.y,
                    linePnt.position.z);

                linePnt.position = tempPnt;
                if (j == obj.points.size() -1)
                {
                    ROS_INFO("RoadLinesCallback, line %d, point %d -> %f,%f,%f",
                        obj.id, j,
                        linePnt.position.x, linePnt.position.y,
                        linePnt.position.z);
                }
                obj.points.at(j) = linePnt;
            }
        }
        mRoadLineMap.insert(std::pair<int, route_mission_handler::RoadLine>(
            obj.id,
            obj));
    }
    ROS_INFO("RoadLinesCallback done, mRoadLineMap size %ld",
        mRoadLineMap.size());
}

void LaneConstructor::RoadMarkersCallback(
    const route_mission_handler::MarkerPolygonArray::ConstPtr &msg)
{
    ROS_INFO("RoadMarkersCallback marker num %ld", msg->markers.size());
    for (int i=0 ; i < msg->markers.size(); i++)
    {
        route_mission_handler::MarkerPolygon obj = msg->markers.at(i);
        if (abs(obj.center_point.x) > 1000000 &&
            abs(obj.center_point.y) > 1000000)
        {
            // transform ecef to local cartesian
            geometry_msgs::Point tempPnt = TransformLocalCartesian(
                obj.center_point.x,
                obj.center_point.y,
                obj.center_point.z);
            obj.center_point = tempPnt;
        }
        if (obj.corner_points.size() > 0 &&
            abs(obj.corner_points.at(0).x) > 1000000)
        {
            std::vector<geometry_msgs::Point> convertedPnts;
            for (auto pnt: obj.corner_points)
            {
                geometry_msgs::Point tempPnt = TransformLocalCartesian(
                    pnt.x,
                    pnt.y,
                    pnt.z);
                convertedPnts.push_back(tempPnt);
            }
            obj.corner_points = convertedPnts;
        }

        std::string key = std::to_string(obj.id);

        if (!mRoadMarkerMap.count(key))
        {
            mRoadMarkerMap.insert(
                std::pair<std::string, route_mission_handler::MarkerPolygon>(
                    key, obj));
        }
        for (auto pair: obj.pairs)
        {
            if (!mLaneMarkers.count(pair.laneId))
            {
                mLaneMarkers.insert(std::pair<int, std::set<std::string>>(
                    pair.laneId, std::set<std::string>()));
            }
            mLaneMarkers.at(pair.laneId).insert(key);
        }
    }
    ROS_INFO("RoadMarkersCallback size mRoadMarkerMap %ld, mLaneMarkers %ld",
        mRoadMarkerMap.size(), mLaneMarkers.size());
}

void LaneConstructor::GetBindingLaneFromNavgRoad(
    route_mission_handler::MarkerPolygon &marker,
    std::vector<int> &laneIds,
    std::map<int, LaneNavg> &laneNavgMap)
{
    if (laneIds.size() > 0)
    {
        for (auto laneId: laneIds)
        {
            if (laneNavgMap.count(laneId))
            {
                if (laneNavgMap.at(laneId).navgroad1_id == marker.navgId &&
                    laneNavgMap.at(laneId).navgroad2_id == marker.navgId)
                {
                    route_mission_handler::Pair pair;
                    pair.laneId = laneId;
                    marker.pairs.push_back(pair);
                    ROS_INFO("GetBindingLaneFromNavgRoad update pair with "
                        "laneId %d, pairs size %ld",
                        marker.pairs.at(marker.pairs.size() - 1).laneId,
                        marker.pairs.size());
                }
                else
                {
                    ROS_INFO("GetBindingLaneFromNavgRoad laneNavgMap.at(%d) "
                        "navgroad1_id %d, navgroad2_id %d",
                        laneId,
                        laneNavgMap.at(laneId).navgroad1_id,
                        laneNavgMap.at(laneId).navgroad2_id);
                }
            }
            else
            {
                ROS_WARN("GetBindingLaneFromNavgRoad laneNavgMap not found %d",
                    laneId);
            }
        }
    }
}

void LaneConstructor::NonAccessiblesCallback(
    const route_mission_handler::MarkerPolygonArray::ConstPtr &msg)
{
    for (int i=0 ; i < msg->markers.size(); i++)
    {
        route_mission_handler::MarkerPolygon obj = msg->markers.at(i);
        ROS_INFO("NonAccessiblesCallback marker %d(%d), navgId %d, "
            "pairs size %ld, corner_points size %ld",
            obj.id, obj.type, obj.navgId,
            obj.pairs.size(),
            obj.corner_points.size());

        if (obj.type == static_cast<int>(LaneType::ZEBRACROSS) &&
            obj.navgId > 0 &&
            obj.pairs.size() == 0)
        {
            if (mNavgRoadMap.count(obj.navgId))
            {
                GetBindingLaneFromNavgRoad(
                    obj,
                    mNavgRoadMap.at(obj.navgId).positiveLaneIds,
                    mLaneNavgMap);

                GetBindingLaneFromNavgRoad(
                    obj,
                    mNavgRoadMap.at(obj.navgId).negativeLaneIds,
                    mLaneNavgMap);
            }
            else
            {
                ROS_WARN("NonAccessiblesCallback mNavgRoadMap not found %d",
                    obj.navgId);
            }
        }

        if (obj.corner_points.size() > 0 &&
            abs(obj.corner_points.at(0).x) > 1000000)
        {
            std::vector<geometry_msgs::Point> convertedPnts;
            for (auto pnt: obj.corner_points)
            {
                geometry_msgs::Point tempPnt = TransformLocalCartesian(
                    pnt.x,
                    pnt.y,
                    pnt.z);
                convertedPnts.push_back(tempPnt);
            }
            obj.corner_points = convertedPnts;
        }

        std::string key = std::to_string(obj.type) + "_"+ std::to_string(obj.id);
        if (!mRoadMarkerMap.count(key))
        {
            mRoadMarkerMap.insert(
                std::pair<std::string, route_mission_handler::MarkerPolygon>(
                    key, obj));
        }
        for (auto pair: obj.pairs)
        {
            if (!mLaneMarkers.count(pair.laneId))
            {
                mLaneMarkers.insert(std::pair<int, std::set<std::string>>(
                    pair.laneId, std::set<std::string>()));
            }
            mLaneMarkers.at(pair.laneId).insert(key);
        }
    }
    ROS_INFO("NonAccessiblesCallback size mRoadMarkerMap %ld, mLaneMarkers %ld",
        mRoadMarkerMap.size(), mLaneMarkers.size());
}

void LaneConstructor::ParkingSpacesCallback(
    const route_mission_handler::ParkingSpaceArray::ConstPtr &msg)
{
    ROS_INFO("ParkingSpacesCallback, space size %ld", msg->spaces.size());
    for (int i=0 ; i < msg->spaces.size(); i++)
    {
        route_mission_handler::ParkingSpace obj = msg->spaces.at(i);
        if (obj.points.size() > 0 &&
            abs(obj.points.at(0).x) > 1000000)
        {
            std::vector<geometry_msgs::Point> convertedPnts;
            for (auto pnt: obj.points)
            {
                geometry_msgs::Point tempPnt = TransformLocalCartesian(
                    pnt.x,
                    pnt.y,
                    pnt.z);
                convertedPnts.push_back(tempPnt);
            }
            if (convertedPnts.size() > 0)
            {
                obj.points = convertedPnts;
            }
        }
        mParkingSpaces.insert(
            std::pair<int, route_mission_handler::ParkingSpace>(
                obj.id,
                obj));
        if (!mLaneParkingSpaceMap.count(obj.laneId))
        {
            mLaneParkingSpaceMap.insert(std::pair<int, std::vector<int>>(
                obj.laneId,
                std::vector<int>()));
        }
        mLaneParkingSpaceMap.at(obj.laneId).push_back(obj.id);
    }
    ROS_INFO("ParkingSpacesCallback size mParkingSpaces %ld, mLaneMarkers %ld",
        mParkingSpaces.size(), mLaneParkingSpaceMap.size());
}

void LaneConstructor::TrafficLightCallback(
    const route_mission_handler::TrafficLightArray::ConstPtr &msg)
{
    mTrafficLightMap.clear();
    for (int i=0; i < msg->lights.size(); i++)
    {
        route_mission_handler::TrafficLight light = msg->lights.at(i);
        for (int j=0; j < light.points.size(); j++)
        {
            if (abs(light.points.at(j).x) > 1000000 &&
                abs(light.points.at(j).y) > 1000000)
            {
                geometry_msgs::Point tempPnt = TransformLocalCartesian(
                    light.points.at(j).x,
                    light.points.at(j).y,
                    light.points.at(j).z);
                light.points.at(j)= tempPnt;
            }
        }

        if (!mTrafficLightMap.count(light.id))
        {
             mTrafficLightMap.insert(
                 std::pair<int, route_mission_handler::TrafficLight>(
                     light.id,
                     light));
        }
        else
        {
            ROS_WARN("mTrafficLightMap already has groud %d", light.id);
        }
    }
    ROS_INFO("TrafficLightCallback, mTrafficLightMap size %ld",
        mTrafficLightMap.size());
}

void LaneConstructor::LaneInfoCallback(
    const route_mission_handler::LanesArray::ConstPtr &msg)
{
    ROS_INFO("LaneInfoCallback, LaneArray lanes size %ld",
        msg->lanes.size());
    for (int i=0; i < msg->lanes.size(); i++)
    {
        route_mission_handler::Lanes lane = msg->lanes.at(i);
        ROS_INFO("LaneInfoCallback, lane %d size of nexts %ld, isTurn %d, "
            "roadLineIds %ld ",
            lane.id, lane.nexts.size(), lane.isTurn, lane.roadLineIds.size());
        if (mLanes.count(lane.id))
        {
            mLanes.at(lane.id).isTurn = lane.isTurn;
            mLanes.at(lane.id).laneno = lane.laneno;
            mLanes.at(lane.id).nexts = lane.nexts;
            mLanes.at(lane.id).roadLineIds = lane.roadLineIds;
            ROS_INFO("LaneInfoCallback, update lane %d", lane.id);
        }
        else
        {
            mLanes.insert(
                std::pair<int, route_mission_handler::Lanes>(lane.id, lane));
        }

        if (!mLanePoints.count(lane.id))
        {
            mLanePoints.insert(std::pair<int, std::vector<std::string>>(
                lane.id,
                std::vector<std::string>()));
        }
    }
    ROS_INFO("LaneInfoCallback done, mLanePoints size %ld", mLanePoints.size());
}

std::vector<route_mission_handler::Lane> LaneConstructor::GetLanes(
    std::vector<int> &laneIds,
    route_mission_handler::Croad &croad,
    int nextCroadId,
    bool isPositive2,
    std::vector<route_mission_handler::Lane> &lanes)
{
    std::vector<route_mission_handler::Lane> results;
    ROS_INFO("GetLanes laneIds %d", laneIds.size());
    for (int i=0; i < laneIds.size(); i++)
    {
        int laneId = laneIds.at(i);
        if (!mLaneNavgMap.count(laneId))
        {
            ROS_WARN("GetLanes mLaneNavgMap no found %d", laneId);
            continue;
        }

        if (!mLanePoints.count(laneId))
        {
            ROS_WARN("GetLanes mLanePoints no found %d", laneId);
            continue;
        }
        LaneNavg laneNavg =  mLaneNavgMap.at(laneId);
        if (laneNavg.navgroad1_id == croad.nroad_id &&
            laneNavg.isPositive1 == croad.isPositive &&
            laneNavg.navgroad2_id == nextCroadId &&
            laneNavg.isPositive2 == isPositive2)
        {
            route_mission_handler::Lane lane;
            lane.lane_id = laneId;
            lane.nroad_ids.push_back(croad.nroad_id);
            lane.nroad_ids.push_back(nextCroadId);

            if (mLanes.count(laneId))
            {
                lane.next_lanes = mLanes.at(laneId).nexts;
                lane.is_intersection = mLanes.at(laneId).isTurn;
            }
            else
            {
                ROS_WARN("GetLanes mLanes not found %d", laneId);
            }

            for (int p = 0; p < mLanePoints.at(laneId).size(); p++)
            {
                std::string key = mLanePoints.at(laneId).at(p);
                if (mPointsMap.count(key))
                {
                    route_mission_handler::Waypoints origWaypnt = mPointsMap.at(key);
                    route_mission_handler::Waypoint waypnt;
                    waypnt.lane_id = laneId;
                    waypnt.point_id = origWaypnt.pointId;
                    waypnt.roadWidth = origWaypnt.roadWidth;
                    waypnt.distToRightLine = origWaypnt.distToRightLine;
                    waypnt.distToLeftLine = origWaypnt.distToLeftLine;
                    waypnt.heading = (origWaypnt.pose.orientation.w) *
                        (M_PI / 180);
                    waypnt.curvature = origWaypnt.curve;
                    waypnt.point = origWaypnt.pose.position;
                    waypnt.markerIds = origWaypnt.markerIds;
                    waypnt.markerTypes = origWaypnt.markerTypes;
                    waypnt.nroad_id = GetNearCroad(
                        waypnt,
                        mLaneNavgMap.at(laneId),
                        mNavgRoadMap);
                    lane.waypoints.push_back(waypnt);
                }
                else
                {
                    ROS_WARN("GetLanes mPointsMap not found %s",
                        key.c_str());
                }
            }

            std::vector<LaneNavg> lns;
            FindSideLanesByValue(lns, mLaneNavgMap, laneNavg);
            ROS_INFO("GetLanes lane %d, find side lanes size %ld",
                lane.lane_id, lns.size());
            for(auto iter: lns)
            {
                if (laneNavg.laneOrder < iter.laneOrder)
                {
                    lane.left_lanes.push_back(iter.laneId);
                    ROS_INFO("GetLanes left_lanes add %d, size %ld",
                        iter.laneId, lane.left_lanes.size());
                }
                else if (laneNavg.laneOrder > iter.laneOrder)
                {
                    lane.right_lanes.push_back(iter.laneId);
                    ROS_INFO("GetLanes right_lanes add %d, size %ld",
                        iter.laneId, lane.right_lanes.size());
                }
#ifdef DEBUG
                else
                {
                    ROS_WARN("Not match! iter %d, order %d, navgroad1_id %d(%d),"
                        "navgroad2_id %d(%d)",
                        iter.laneId,
                        iter.laneOrder,
                        iter.navgroad1_id,
                        iter.isPositive1,
                        iter.navgroad2_id,
                        iter.isPositive2);
                }
#endif
            }

            ROS_INFO("GetLanes add lane %d, left_lanes size %ld, right_lanes size %ld",
                lane.lane_id, lane.left_lanes.size(), lane.right_lanes.size());
            results.push_back(lane);
            croad.lane_ids.push_back(laneId);
        }
#ifdef DEBUG
        else
        {
            ROS_WARN("GetLanes lane %d, navgroad1 %d, isPositive1 %d, "
                "navgroad2 %d, isPositive2 %d, no match,  skip!!",
                laneId,
                mLaneNavgMap.at(laneId).navgroad1_id,
                mLaneNavgMap.at(laneId).isPositive1,
                mLaneNavgMap.at(laneId).navgroad2_id,
                mLaneNavgMap.at(laneId).isPositive2);
        }
#endif
    }
    return results;
}

void LaneConstructor::LoadCompletedCallback(const std_msgs::Bool::ConstPtr &msg)
{
    ros::Rate loop_rate(100);
    ROS_INFO("LoadCompletedCallback");
    route_mission_handler::Path path;
    path.name = mRoute;
    std::map<int, route_mission_handler::Lane> displayLanes;
    std::map<int, route_mission_handler::Croad> displayNroads;

    for (int i=0; i < mRouteNavgcroadOrders.size(); i++)
    {
        int croadId = mRouteNavgcroadOrders.at(i);
        bool isPositive = mRouteNavgcroads.at(croadId);
        int nextCroadId = -1;
        bool isPositive2 = false;
        if (i + 1 < mRouteNavgcroadOrders.size())
        {
            nextCroadId = mRouteNavgcroadOrders.at(i + 1);
            isPositive2 = mRouteNavgcroads.at(nextCroadId);
        }

        ROS_INFO("LoadCompletedCallback croadId %d, isPositive %d, "
            "nextCroadId %d, isPositive2 %d",
            croadId, isPositive, nextCroadId, isPositive2);
        if (!mNavgRoadMap.count(croadId))
        {
            ROS_WARN("LoadCompletedCallback mNavgRoadMap not found %d", croadId);
            continue;
        }
        route_mission_handler::NavgRoad nr =  mNavgRoadMap.at(croadId);
        route_mission_handler::Croad croad;
        croad.nroad_id = nr.id;
        croad.isPositive = isPositive;
        for (int g=0; g < nr.points.size(); g++)
        {
            croad.points.push_back(nr.points.at(g).pose.position);
        }
        ROS_INFO("LoadCompletedCallback croad.points size %ld",
            croad.points.size());

        std::vector<route_mission_handler::Lane> lanes;
        if (isPositive)
        {
            lanes = GetLanes(
                nr.positiveLaneIds,
                croad,
                nextCroadId,
                isPositive2,
                path.lanes);
        }
        else
        {
            lanes = GetLanes(
                nr.negativeLaneIds,
                croad,
                nextCroadId,
                isPositive2,
                path.lanes);
            std::reverse(croad.points.begin(), croad.points.end());
        }
        ROS_INFO("LoadCompletedCallback lanes size %ld", lanes.size());

        int nroadId = croad.nroad_id;
        if (std::find_if(
            path.nroads.begin(),
            path.nroads.end(),
            [&nroadId](const route_mission_handler::Croad &n)
            {
                return n.nroad_id == nroadId;
            }) == path.nroads.end())
        {
            path.nroads.push_back(croad);
            ROS_INFO("LoadCompletedCallback path.nroads add croad %d, size %ld",
                croad.nroad_id, path.nroads.size());
        }

        for (int j = 0; j < lanes.size(); j++)
        {
             int laneId = lanes.at(j).lane_id;
             path.lanes.push_back(lanes.at(j));
             ROS_INFO("LoadCompletedCallback path.lanes add candidata lane %d, size %ld",
                 laneId, path.lanes.size());
        }
    }

    for (int k=0; k < path.nroads.size(); k++)
    {
        route_mission_handler::Croad croad = path.nroads.at(k);
        displayNroads.insert(std::pair<int, route_mission_handler::Croad>(
            croad.nroad_id,
            croad));
#ifdef DEBUG
        std::string laneLog = "";
        for (int k2 = 0; k2 < croad.lane_ids.size(); k2++)
        {
            laneLog += std::to_string(croad.lane_ids.at(k2)) + " ";
        }
        ROS_INFO("LoadCompletedCallback final croad %d (%d), lanes: %s, point size %ld",
            croad.nroad_id, croad.isPositive, laneLog.c_str(),
            croad.points.size());
#endif
    }

    for (int h=0; h < path.lanes.size(); h++)
    {
        route_mission_handler::Lane lane = path.lanes.at(h);
        ROS_INFO("LoadCompletedCallback lane %d , next lanes size %ld",
            lane.lane_id, lane.next_lanes.size());
        for (auto nextLaneId: lane.next_lanes)
        {
          if (std::find_if(
              path.lanes.begin(),
              path.lanes.end(),
              [&nextLaneId](const route_mission_handler::Lane &n)
              {
                  return n.lane_id == nextLaneId;
              }) == path.lanes.end())
          {
              RemoveIntFromVector(lane.next_lanes, nextLaneId);
              ROS_WARN("LoadCompletedCallback next lane not in path, remove it %d, remind size %ld",
                  nextLaneId, lane.next_lanes.size());
          }
        }
        displayLanes.insert(std::pair<int, route_mission_handler::Lane>(
            lane.lane_id,
            lane));
#ifdef DEBUG
        std::string laneLog = "nroad_ids:";
        std::string pntLog = "";

        for (int h2 = 0; h2 < lane.nroad_ids.size(); h2++)
        {
            laneLog += std::to_string(lane.nroad_ids.at(h2)) + " ";
        }
        laneLog += ", next_lanes:";
        for (int h3 = 0; h3 < lane.next_lanes.size(); h3++)
        {
            laneLog += std::to_string(lane.next_lanes.at(h3)) + " ";
        }
        laneLog += ", is_intersection:" + std::to_string(lane.is_intersection);

        // for (int h4 = 0; h4 < lane.waypoints.size(); h4++)
        // {
        //     route_mission_handler::Waypoint waypnt = lane.waypoints.at(h4);
        //     pntLog += "laneId:" + std::to_string(waypnt.lane_id) +
        //         ", pointId:" + std::to_string(waypnt.point_id);
        //     pntLog += ", markerIds:";
        //     for (int m = 0; m < waypnt.markerIds.size(); m++)
        //     {
        //         pntLog += std::to_string(waypnt.markerIds.at(m)) + " ";
        //     }
        //     pntLog += ", markerTypes:";
        //     for (int t = 0; t < waypnt.markerTypes.size(); t++)
        //     {
        //         pntLog += std::to_string(waypnt.markerTypes.at(t)) + " ";
        //     }
        //
        //     pntLog += ", nroad_id:" + std::to_string(waypnt.nroad_id) +
        //         ", width:" + std::to_string(waypnt.roadWidth) +
        //         ", distToRightLine:" + std::to_string(waypnt.distToRightLine) +
        //         ", distToLeftLine:" + std::to_string(waypnt.distToLeftLine) +
        //         ", curve:" + std::to_string(waypnt.curvature) +
        //         ", heading:" + std::to_string(waypnt.heading) +
        //         ", point:" + std::to_string(waypnt.point.x) + "," +
        //         std::to_string(waypnt.point.y) + "," +
        //         std::to_string(waypnt.point.z);
        //     pntLog += "\n";
        // }

        ROS_INFO("LoadCompletedCallback final lane %d, %s, waypoints size %ld, left_lanes "
            "size %ld, right_lanes size %ld, next_lanes %ld ->%s\n",
            lane.lane_id,
            laneLog.c_str(),
            lane.waypoints.size(),
            lane.left_lanes.size(),
            lane.right_lanes.size(),
            lane.next_lanes.size(),
            pntLog.c_str());
#endif
    }

    mDisplayMgr->DisplayWaypointMarkers(mPointsMap, displayLanes, mLanePoints);
    mDisplayMgr->DisplayRoadLines(mRoadLineMap);
    mDisplayMgr->DisplayRoadMarkers(mRoadMarkerMap);
    mDisplayMgr->DisplayParkingSpaces(mParkingSpaces);
    mDisplayMgr->DisplayTrafficLights(mTrafficLightMap);
    mDisplayMgr->DisplayNavgRoads(displayNroads);

    while(ros::ok())
    {
       mPubFinal.publish(path);
       loop_rate.sleep();
    }
}
