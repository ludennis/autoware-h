#!/usr/bin/python2.7

import rospy
import rospkg

from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from route_mission_handler.msg import Pair
from route_mission_handler.msg import ItriPoint
from route_mission_handler.msg import MarkerPolygon
from route_mission_handler.msg import MarkerPolygonArray
from route_mission_handler.msg import Paths
from route_mission_handler.msg import PathsArray
from route_mission_handler.msg import Lanes
from route_mission_handler.msg import LanesArray
from route_mission_handler.msg import Waypoints
from route_mission_handler.msg import NavgRoad
from route_mission_handler.msg import NavgRoadArray
from route_mission_handler.msg import LaneNavgRoad
from route_mission_handler.msg import LaneNavgRoadArray
from route_mission_handler.msg import ConnectedNavgRoad
from route_mission_handler.msg import RoadLineArray
from route_mission_handler.msg import RoadLine
from route_mission_handler.msg import RoadLinePoint
from route_mission_handler.msg import ParkingSpace
from route_mission_handler.msg import ParkingSpaceArray
from route_mission_handler.msg import ParkingLot
from route_mission_handler.msg import ParkingLotArray
from route_mission_handler.msg import TrafficLight
from route_mission_handler.msg import TrafficLightArray
import numbers
import json
import os
import numpy as np
import time

MarkerType = ["WHITE_DASHED_LINE",
    "WHITE_SOLID_LINE",
    "WHITE_DOUBLE_LINE",
    "YELLOW_DASHED_LINE",
    "YELLOW_SOLID_LINE",
    "YELLOW_DOUBLE_LINE",
    "RED_SOLID_LINE", # 6
    "STRAIGHT_ARROW",
    "TURN_LEFT_ARROW",
    "TURN_RIGHT_ARROW",
    "STRAIGHT_OR_LEFT_ARROW", # 10
    "STRAIGHT_OR_RIGHT_ARROW",
    "TURN_RIGHT_ONLY",
    "STOP_LINE",
    "LONGER_BUMP",
    "SHORTER_BUMP", # 15
    "SLOWDOWN",
    "YIELD",
    "STOP_SIGN",
    "SPEED_LIMIT_20KPH", # 19
    "SPEED_LIMIT_30KPH",
    "SPEED_LIMIT_40KPH",
    "SPEED_LIMIT_50KPH",
    "SCOOTER_PARKING_PLACE",
    "PARKING_SPACE",
    "ZEBRACROSS", # 25
    "INTERSECTION_AREA",
    "NON_DRIVING_AREA",
    "INTERSECTION_POINT",
    "LOW_SPEED",
    "LOW_SPEED_LINE", # 30
    "YIELD_LINE",
    "NO_TEMPPARKING_AREA",
    "TEMPPARKING_AREA",
    "MAX_SPEED_LIMIT",
    "PARKING_WARNING", # 35
    "ROAD_WARNING",
    "SPEED_WARNING",
    "FIRE_HYDRANT",
    "ROAD_GUIDE",
    "NO_PARKING", # 40
    "TURNUNG_LINE",
    "DRIVING_DRIECTIONS",
    "STOP_FOR_INSPECTION",
    "WIGHT_RESTRICTION",
    "NO_CHANGING_LANES", # 45
    "NO_ENTRY",
    "PROHIBIT_RIGHT",
    "BIKE_LANE",
    "SPEED_LIMIT_40KPH",
    "TRAFFIC_LIGTH", # 50
    "PEDESTRIAN_LIGTH",
    "BOUNDARIES",
    "VIRTUAL_LINE",
    "TRIPLE_ARROW",
    "HSR_STATION"]

class FileManager():
    def __init__(self, folder, publisher):
        self.folder = folder
        self.publisher = publisher

    def GetWaypointData(self, waypoints_info):
        waypoint = Waypoints()
        if 'point_id' in waypoints_info:
            waypoint.pointId = waypoints_info['point_id']
        # elif 'id' in waypoints_info:
        #     waypoint.pointId = waypoints_info['id']

        if 'x' in waypoints_info:
            waypoint.pose.position.x = waypoints_info['x']
        if 'y' in waypoints_info:
            waypoint.pose.position.y = waypoints_info['y']
        if 'z' in waypoints_info:
            waypoint.pose.position.z = waypoints_info['z']

        if 'heading' in waypoints_info:
            waypoint.pose.orientation.w =  waypoints_info['heading']
        elif 'angle' in waypoints_info:
            if waypoints_info['angle'] is None:
                waypoint.pose.orientation.w =  0.0
            else:
                waypoint.pose.orientation.w =  waypoints_info['angle']

        if 'leftline_id' in waypoints_info:
            waypoint.leftLineId = waypoints_info['leftline_id']
        else:
            waypoint.leftLineId = -1
        if 'leftpoint_id' in waypoints_info:
            waypoint.leftLinePointId = waypoints_info['leftpoint_id']
        else:
            waypoint.leftLinePointId = -1

        if 'rightline_id' in waypoints_info:
            waypoint.rightLineId = waypoints_info['rightline_id']
        else:
            waypoint.rightLineId = -1
        if 'rightpoint_id' in waypoints_info:
            waypoint.rightLinePointId = waypoints_info['rightpoint_id']
        else:
            waypoint.rightLinePointId = -1

        if 'road_width' in waypoints_info:
            waypoint.roadWidth = waypoints_info['road_width']
        elif 'width' in waypoints_info:
            waypoint.roadWidth = waypoints_info['width']
        else:
            waypoint.roadWidth = 2.0

        if 'distToLeftLine' in waypoints_info:
            waypoint.distToLeftLine = waypoints_info['distToLeftLine']
        elif 'distance_to_left boundary' in waypoints_info:
            waypoint.distToLeftLine = waypoints_info['distance_to_left boundary']
        else:
            waypoint.distToLeftLine = 1.0

        if 'distToRightLine' in waypoints_info:
            waypoint.distToRightLine = waypoints_info['distToRightLine']
        elif 'distance_to_right boundary' in waypoints_info:
            waypoint.distToRightLine = waypoints_info['distance_to_right boundary']
        else:
            waypoint.distToRightLine = 1.0

        if 'markerId' in waypoints_info:
            waypoint.markerIds.append(int(waypoints_info['markerId']))
        elif 'road_marker_id' in waypoints_info:
            if waypoints_info['road_marker_id'] != None:
                if isinstance(waypoints_info['road_marker_id'], list):
                    for tmp in waypoints_info['road_marker_id']:
                        waypoint.markerIds.append(int(tmp))
                else:
                    waypoint.markerIds.append(int(waypoints_info['road_marker_id']))
            # elif waypoints_info['road_marker_id'] == ' ' or waypoints_info['road_marker_id'] == None:
            #     rospy.logwarn('road_marker_id none')
            # else:
            #     rospy.logwarn('road_marker_id is null')

        if 'markerType' in waypoints_info:
            waypoint.markerTypes.append(int(waypoints_info['markerType']))
        elif 'road_marker_type' in waypoints_info:
            if waypoints_info['road_marker_type'] != None:
                # print(waypoints_info['road_marker_type'])
                if isinstance(waypoints_info['road_marker_type'], list):
                    for tmp in waypoints_info['road_marker_type']:
                        waypoint.markerTypes.append(int(tmp))
                else:
                    waypoint.markerTypes.append(int(waypoints_info['road_marker_type']))
            else:
                rospy.logwarn('road_marker_type is null')

        if 'curve' in waypoints_info:
            waypoint.curve = waypoints_info['curve']
        elif 'curvature' in waypoints_info:
            if waypoints_info['curvature'] is None:
                waypoint.curve = 0.0
            elif isinstance(waypoints_info['curvature'], numbers.Number):
                waypoint.curve = float(waypoints_info['curvature'])
            elif 'imag' in waypoints_info['curvature']:
                waypoint.curve = float(waypoints_info['curvature']['imag'])

        else:
            waypoint.curve = 0.0

        # if 'lat' in waypoints_info:
        #     waypoint.geopoint.latitude = waypoints_info['lat']
        # else:
        #     waypoint.geopoint.latitude = 0.0
        #
        # if 'lng' in waypoints_info:
        #     waypoint.geopoint.longitude = waypoints_info['lng']
        # else:
        #     waypoint.geopoint.longitude = 0.0

        if 'bank_angle' in waypoints_info:
            if waypoints_info['bank_angle'] is None:
                waypoint.bank_angle = 0.0
            elif waypoints_info['bank_angle'] == ' ':
                waypoint.bank_angle = 0.0
            else:
                waypoint.bank_angle = waypoints_info['bank_angle']
        else:
            waypoint.bank_angle = 0.0

        if 'slope' in waypoints_info:
            if waypoints_info['slope'] is None:
                waypoint.slope = 0.0
            else:
                waypoint.slope = waypoints_info['slope']
        else:
            waypoint.slope = 0.0
        return waypoint

    def ReadChangedWaypointsFile(self, fileIndex, jsonData):
        path = Paths()
        laneId = 0
        for globalpath_index in range(len(jsonData['waypoints'])):
            if 'global_path' in jsonData['waypoints'][globalpath_index]:
                for path_info in jsonData['waypoints'][globalpath_index]['global_path']:
                    lane = Lanes()
                    if 'lane_id' in path_info:
                        laneId = int(path_info['lane_id'])
                    else:
                        laneId += 1
                    lane.id = laneId

                    for waypoints_info in path_info['points']:
                        waypoint = self.GetWaypointData(waypoints_info)
                        waypoint.laneId = laneId
                        lane.waypoints.append(waypoint)
                    rospy.loginfo("ReadChangedWaypointsFile 1, lane %d, points len %d", lane.id, len(lane.points))
                    path.lanes.append(lane)
            else:
                waypoints_info = jsonData['waypoints'][globalpath_index]
                lane = Lanes()
                if 'lane_id' in waypoints_info:
                    laneId = int(waypoints_info['lane_id'])
                else:
                    laneId += 1
                lane.id = laneId
                if 'points' in waypoints_info:
                    for point_index in range(len(waypoints_info['points'])):
                        waypoint = self.GetWaypointData(waypoints_info['points'][point_index])
                        waypoint.laneId = laneId
                        # if waypoint.pointId % 10 == 0:
                        #     rospy.loginfo("ReadChangedWaypointsFile, waypoint lane %d, pointId %d, x,y,z %f,%f,%f, curve %f",
                        #         waypoint.laneId, waypoint.pointId, waypoint.pose.position.x, waypoint.pose.position.y,waypoint.pose.position.z, waypoint.curve)
                        # path.waypoints.append(waypoint)
                        lane.points.append(waypoint)
                else:
                    waypoint = self.GetWaypointData(waypoints_info)
                    waypoint.laneId = laneId
                    lane.points.append(waypoint)
                    rospy.loginfo("ReadChangedWaypointsFile, waypoint lane %d, pointId %d, x,y,z %f,%f,%f, curve %f",
                        waypoint.laneId, waypoint.pointId, waypoint.pose.position.x, waypoint.pose.position.y,waypoint.pose.position.z, waypoint.curve)
                    # path.waypoints.append(waypoint)
                rospy.loginfo("ReadChangedWaypointsFile 2, lane %d, points len %d", lane.id, len(lane.points))
                path.lanes.append(lane)

        rospy.loginfo("ReadChangedWaypointsFile, path.lanes size %ld", len(path.lanes))
        return path

    def ReadNavgRoadsFile(self, jsonData):
        navgroadArray = NavgRoadArray()
        rospy.loginfo("ReadNavgRoadsFile, len(jsonData['navgroads'] %ld", len(jsonData['navgroads']))
        cnt = 0
        for navgroad_index in range(len(jsonData['navgroads'])):
            navgroad = NavgRoad()
            cnt +=1
            for path_info in jsonData['navgroads'][navgroad_index]:
                data = jsonData['navgroads'][navgroad_index][path_info]
                if path_info == 'id':
                    navgroad.id = data

                if path_info == "pos_lane_num":
                    navgroad.positiveLaneNum = data

                if path_info == "neg_lane_num":
                    navgroad.negativeLaneNum = data

                if path_info == "length":
                    navgroad.length = float(data)

                if path_info == "mode":
                    navgroad.mode = data
                if path_info == "n_as":
                    navgroad.n_as = data
                if path_info == "n_kph":
                    navgroad.n_kph = data
                if path_info == "p_as":
                    navgroad.p_as = data
                if path_info == "p_kph":
                    navgroad.p_kph = data

                if path_info == "two_way":
                    navgroad.twoWay = data
                elif path_info == 'direction':
                    navgroad.twoWay = data

                if path_info == "pos_lanes":
                    lanes = data.split(',')
                    if lanes[0] !='':
                        for lane_id in lanes:
                            navgroad.positiveLaneIds.append(int(lane_id))

                if path_info == "neg_lanes":
                    lanes = data.split(',')
                    if lanes[0] !='':
                        for lane_id in lanes:
                            navgroad.negativeLaneIds.append(int(lane_id))

                if path_info == "heads" or path_info == "head_point":
                    if isinstance(data, list):
                        for heads_info in data:
                            # print(heads_info)
                            if heads_info != None and heads_info != 'nan':
                                nr = ConnectedNavgRoad()
                                nr.id = int(heads_info)
                                navgroad.heads.append(nr)
                    else:
                        for heads_info in data:
                            nr = ConnectedNavgRoad()
                            nr.id = -1
                            if 'id' in heads_info:
                                nr.id = heads_info['id']
                                nr.pointId = heads_info['point_id']
                                nr.length= heads_info['len']
                            elif heads_info != '' and heads_info != ' ':
                                if isinstance(heads_info, str):
                                    nr.id = int(heads_info)
                                else:
                                    print(heads_info)
                                    rospy.logwarn("ReadNavgRoadsFile, heads_info not str type")
                            if nr.id >= 0:
                                navgroad.heads.append(nr)

                if path_info == "tails" or path_info == "end_point":
                    if isinstance(data, list):
                        for tails_info in data:
                            # print(tails_info)
                            if tails_info != None and tails_info != 'nan':
                                nr = ConnectedNavgRoad()
                                nr.id = int(tails_info)
                                navgroad.tails.append(nr)
                    else:
                        for tails_info in data:
                            nr = ConnectedNavgRoad()
                            nr.id = -1
                            rospy.loginfo("ReadNavgRoadsFile, tails_info %s!", tails_info)
                            if 'id' in tails_info:
                                nr.id = tails_info['id']
                                nr.pointId = tails_info['point_id']
                                nr.length= tails_info['len']
                            elif tails_info != '' and tails_info != ' ':
                                if isinstance(tails_info, str):
                                    nr.id = int(tails_info)
                                else:
                                    print(tails_info)
                                    rospy.logwarn("ReadNavgRoadsFile, tails_info not str type")
                            if nr.id >= 0:
                                navgroad.tails.append(nr)

                if path_info == "points":
                    tempNavgId = -1
                    for point_info in data:
                        # rospy.loginfo("ReadNavgRoadsFile, point_info %s", point_info)
                        point = ItriPoint()
                        if 'nav_croad_id' in point_info:
                            point.belongId = point_info['nav_croad_id']
                            # rospy.loginfo("ReadNavgRoadsFile npoint.belongId %d, navgroad.id %d", point.belongId, navgroad.id)
                            if navgroad.id != point.belongId:
                                if len(navgroad.points) > 0:
                                    navgroadArray.navgroads.append(navgroad)
                                    # rospy.loginfo("ReadNavgRoadsFile navgroadArray add navg %d, point size %ld", navgroad.id, len(navgroad.points))
                                navgroad = NavgRoad()
                                navgroad.id = point.belongId
                                # rospy.loginfo("ReadNavgRoadsFile add new navg %d", navgroad.id)
                        elif navgroad.id == 0:
                            point.belongId = cnt
                        else:
                            point.belongId = navgroad.id
                        point.pointId = point_info['point_id']

                        # ns = NavSatFix()
                        # if 'lat' in point_info:
                        #     point.geopoint.latitude = point_info['lat']
                        # if 'lng' in point_info:
                        #     point.geopoint.longitude = point_info['lng']

                        if 'x' in point_info:
                            point.pose.position.x = point_info['x']
                        if 'y' in point_info:
                            point.pose.position.y = point_info['y']
                        if 'z' in point_info:
                            point.pose.position.z = point_info['z']
                        if 'heading' in point_info:
                            point.pose.orientation.w =  point_info['heading']
                        # rospy.loginfo("route_mission_handler navgroad %d, position %f, %f, %f", navgroad.id, point.pose.position.x, point.pose.position.y, point.pose.position.z)
                        navgroad.points.append(point)
            rospy.loginfo("ReadNavgRoadsFile navgroad %d, points %d", navgroad.id, len(navgroad.points))
            navgroadArray.navgroads.append(navgroad)
        return navgroadArray

    def ReadLanesNavgRoadsFile(self, jsonData):
        laneNavgRoadArray = LaneNavgRoadArray()
        for laneNavg_info in jsonData['lanes_navgroads']:

            lanenavgroad = LaneNavgRoad()
            lanenavgroad.lane_id = laneNavg_info['lane_id']
            if 'navgroad1' in laneNavg_info:
                lanenavgroad.navgroad1 = laneNavg_info['navgroad1']
            elif 'nav_croad1_id' in laneNavg_info:
                lanenavgroad.navgroad1 = laneNavg_info['nav_croad1_id']

            if 'isPositive1' in laneNavg_info:
                lanenavgroad.isPositive1 = laneNavg_info['isPositive1']
            elif 'croad1_positive' in laneNavg_info:
                lanenavgroad.isPositive1 = laneNavg_info['croad1_positive']

            if 'navgroad2' in laneNavg_info:
                lanenavgroad.navgroad2 = laneNavg_info['navgroad2']
            elif 'nav_croad2_id' in laneNavg_info:
                lanenavgroad.navgroad2 = laneNavg_info['nav_croad2_id']

            if 'isPositive2' in laneNavg_info:
                lanenavgroad.isPositive2 = laneNavg_info['isPositive2']
            elif 'croad2_positive' in laneNavg_info:
                lanenavgroad.isPositive2 = laneNavg_info['croad2_positive']

            if 'laneno' in laneNavg_info:
                lanenavgroad.laneno = laneNavg_info['laneno']
            if 'seqner' in laneNavg_info and laneNavg_info['seqner'] != None:
                print(laneNavg_info['seqner'])
                if isinstance(laneNavg_info['seqner'], str):
                    lanenavgroad.seqner = laneNavg_info['seqner']
                else:
                    print("seqner not string")
            laneNavgRoadArray.lanenavgroads.append(lanenavgroad)
        return laneNavgRoadArray

    def ReadLaneInfoFile(self, index, jsonData):
        lane = Lanes()

        if 'lane_id' in jsonData:
            lane.id = jsonData['lane_id']
        elif 'id' in jsonData:
            lane.id = jsonData['id']
        else:
            lane.id = index

        if 'isTurn' in jsonData:
            lane.isTurn = int(jsonData['isTurn']) == 1
        elif 'subtype' in jsonData and jsonData['subtype'] != ' ':
            lane.isTurn = int(jsonData['subtype']) == 1
        else:
            lane.isTurn = False

        if 'direction' in jsonData:
            lane.direction = int(jsonData['direction']) == 1
        else:
            lane.direction = False

        if 'nexts' in jsonData:
            if ',' in jsonData['nexts']:
                nexts = jsonData['nexts'].split(',')
            else:
                nexts = jsonData['nexts'].split()
            for temp in nexts:
                lane.nexts.append(int(temp))
        elif 'next_clane' in jsonData:
            for laneid in jsonData['next_clane']:
                lane.nexts.append(int(laneid))

        if 'laneno' in jsonData:
            lane.laneno = int(jsonData['laneno'])
        else:
            lane.laneno = 0

        if 'roadLineIds' in jsonData:
            roadLineIds = jsonData['roadLineIds'].split()
            for temp in roadLineIds:
                lane.roadLineIds.append(int(temp))
        else:
            if 'lroadline_id' in jsonData:
                lane.roadLineIds.append(int(jsonData['lroadline_id']))
            if 'rroadline_id' in jsonData:
                lane.roadLineIds.append(int(jsonData['rroadline_id']))
        # rospy.loginfo("lane %d, roadLineIds len %d",lane.id, len(lane.roadLineIds))
        return lane

    # def ReadWaypointJsonFile(self, file_index, jsonData):
    #     path = Paths()
    #     res = 1
    #     for i in range(int(len(jsonData['global_path'])/res)):
    #         for position_i in range(res):
    #             pointId = position_i + i*res
    #             waypointJson = (jsonData['global_path'])[pointId]
    #             pntX = waypointJson['x']
    #             pntY = waypointJson['y']
    #             pntZ = waypointJson['z']
    #             heading = waypointJson['heading']

    #             waypoint = Waypoints()
    #             if 'lane_id' in waypointJson:
    #                 waypoint.laneId = waypointJson['lane_id']
    #             else:
    #                 waypoint.laneId = file_index + 1

    #             if 'point_id' in waypointJson:
    #                 waypoint.pointId = waypointJson['point_id']
    #             else:
    #                 waypoint.pointId = pointId

    #             if 'leftline_id' in waypointJson:
    #                 waypoint.leftLineId = waypointJson['leftline_id']
    #             else:
    #                 waypoint.leftLineId = -1

    #             if 'leftpoint_id' in waypointJson:
    #                 waypoint.leftLinePointId = waypointJson['leftpoint_id']
    #             else:
    #                 waypoint.leftLinePointId = -1

    #             if 'rightline_id' in path_info:
    #                 waypoint.rightLineId = path_info['rightline_id']
    #             else:
    #                 waypoint.rightLineId = -1

    #             if 'rightpoint_id' in path_info:
    #                 waypoint.rightLinePointId = path_info['rightpoint_id']
    #             else:
    #                 waypoint.rightLinePointId = -1

    #             if 'road_width' in waypointJson:
    #                 waypoint.roadWidth = waypointJson['road_width']
    #             else:
    #                 waypoint.roadWidth = 2.0

    #             if 'distToLeftLine' in waypointJson:
    #                 waypoint.distToLeftLine = waypointJson['distToLeftLine']
    #             else:
    #                 waypoint.distToLeftLine = 1.0
    #             if 'distToRightLine' in waypointJson:
    #                 waypoint.distToRightLine = waypointJson['distToRightLine']
    #             else:
    #                 waypoint.distToRightLine = 1.0

    #             if 'curve' in waypointJson:
    #                 waypoint.curve = waypointJson['curve']
    #             else:
    #                 waypoint.curve = 1.0

    #             if 'markerId' in waypointJson:
    #                 waypoint.markerId = waypointJson['markerId']
    #             else:
    #                 waypoint.markerId = -1

    #             if 'markerType' in waypointJson:
    #                 waypoint.markerType = waypointJson['markerType']
    #             else:
    #                 waypoint.markerType = -1

    #             waypoint.pose.position.x = pntX
    #             waypoint.pose.position.y = pntY
    #             waypoint.pose.position.z = pntZ
    #             waypoint.pose.orientation.w = heading
    #             path.waypoints.append(waypoint)
    #     return  path

    def GetLanes(self, filedir):
        laneArray = LanesArray()
        json_files, filedir = self.CheckFileOrDir(filedir)

        for file_index in range(len(json_files)):
            with open(filedir + '/' + json_files[file_index]) as json_data:
                print(filedir + '/' + json_files[file_index])
                jsonData = json.load(json_data)
                if 'lanes' in jsonData:
                    for lane_index in range(len(jsonData['lanes'])):
                        lane = self.ReadLaneInfoFile((lane_index + 1), jsonData['lanes'][lane_index])
                        laneArray.lanes.append(lane)

        if len(laneArray.lanes) > 0:
            publisher["PubLanes"].publish(laneArray)
            rospy.loginfo("publish topic [/laneinfo_fromfile]")

    def GetRoadLinesFromFile(self, jsonData):
        lineArray = RoadLineArray()
        lineId = 1
        prevPnt = Point()
        prevPnt.x = 0
        prevPnt.y = 0
        prevPnt.z = 0

        for i in range(int(len(jsonData['roadlines']))):
            dataJson = jsonData['roadlines'][i]
            line = RoadLine()
            if 'id' in dataJson:
                line.id = dataJson['id']
            elif 'roadline_id' in dataJson:
                line.id = dataJson['roadline_id']
            else:
                line.id = lineId
                lineId += 1

            if 'points' not in dataJson:
                print("road line %d not points"%(line.id))
            else:
                points = dataJson['points']
                if points is not None:
                    line.points, prevPnt = self.GetPointsOfRoadLine(points, prevPnt)
            lineArray.lines.append(line)
            # print("lineArray line id %d, points num %d"%(line.id, len(line.points)))
        rospy.loginfo("GetRoadLinesFromFile, lineArray publish lineArray len %d", len(lineArray.lines))
        publisher["PubRoadLine"].publish(lineArray)

    def ParseRoadMarker(self, markerJson):
        marker = MarkerPolygon()
        if 'type' in markerJson:
            marker.type = markerJson['type']
        elif 'category' in markerJson:
            marker.type = markerJson['category']

        if 'id' not in markerJson:
            marker.id = cnt
        else:
            marker.id = markerJson['id']

        if 'lanes' in markerJson and markerJson['lanes'] != None:
            for pairId in range(len(markerJson['lanes'])):
                pair = Pair()
                pairJson = markerJson['lanes'][pairId]
                if 'lane_id' in pairJson:
                    pair.laneId = pairJson['lane_id']
                if 'pointIds' in pairJson and pairJson['pointIds'] is not None:
                    print("pointId num %d"%len(pairJson['pointIds']))
                    for pntId in range(len(pairJson['pointIds'])):
                        pair.pointIds.append(int(pairJson['pointIds'][pntId]))
                print("pair laneId %d, pointIds num %d"%(pair.laneId, len(pair.pointIds)))
                marker.pairs.append(pair)
        elif 'lane_id' in markerJson:
            if isinstance(markerJson['lane_id'], int):
                rospy.loginfo("lane_id is int type %d", markerJson['lane_id'])
            else:
                for laneId in markerJson['lane_id']:
                    pair = Pair()
                    pair.laneId = laneId
                    marker.pairs.append(pair)
        else:
            pair = Pair()
            if 'lane_id' in markerJson:
                pair.laneId = markerJson['lane_id']

            if 'point_ids' in markerJson:
                point_ids = markerJson['point_ids'].split()
                for temp in point_ids:
                    pair.pointIds.append(int(temp))
            elif 'point_id' in markerJson:
                pair.pointIds.append(int(markerJson['point_id']))
            marker.pairs.append(pair)

        # if 'lat' in markerJson:
        #     marker.center_geopoint.latitude = markerJson['lat']
        # else:
        #     marker.center_geopoint.latitude = 0.0
        # # center lat, lng, x, y, z
        # if 'lng' in markerJson:
        #     marker.center_geopoint.longitude = markerJson['lng']
        # else:
        #     marker.center_geopoint.longitude = 0.0

        center_point = markerJson['center_point']
        if 'x' in center_point and center_point['x'] != None:
            marker.center_point.x = float(center_point['x'])
        if 'y' in center_point and center_point['y'] != None:
            marker.center_point.y = float(center_point['y'])
        if 'z' in center_point and center_point['z'] != None:
            marker.center_point.z = float(center_point['z'])

        if 'points' not in markerJson:
            rospy.logwarn("No points key in marker")
        else:
            rect_points = markerJson['points']
            for pid in range(len(rect_points)):
                pnt = Point();
                if 'x' in rect_points[pid]:
                    pnt.x = float(rect_points[pid]['x'])
                if 'y' in rect_points[pid]:
                    pnt.y = float(rect_points[pid]['y'])
                if 'z' in rect_points[pid]:
                    pnt.z = float(rect_points[pid]['z'])
                # print("ParseRoadMarker x %f, y %f, z %f"%(pnt.x, pnt.y, pnt.z))
                marker.corner_points.append(pnt)

                # ns = NavSatFix();
                # if 'lat' in rect_points[pid]:
                #     ns.latitude = rect_points[pid]['lat']
                # if 'lng' in rect_points[pid]:
                #     ns.longitude = rect_points[pid]['lng']
                # if ns.latitude != 0 and ns.longitude != 0:
                #     marker.corner_geopoints.append(ns)

        if 'cloud_points' in markerJson and markerJson['cloud_points'] is not None:
            print("cloud_points num %d"%len(markerJson['cloud_points']))
            points = []
            cloud_points = markerJson['cloud_points']
            for pntId in range(len(cloud_points)):
                x = cloud_points[pntId]['x']
                y = cloud_points[pntId]['y']
                z = cloud_points[pntId]['z']
                r = cloud_points[pntId]['r']
                g = cloud_points[pntId]['g']
                b = cloud_points[pntId]['b']
                rgb = (r << 16 | g << 8 | b);
                if pntId % 20 == 0:
                    print("pntId %d, r %d, g %d, b %d, rgb %d"%(pntId, r, g, b, rgb))
                pt = [x, y, z, rgb]
                points.append(pt)
            fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1), PointField('rgba', 12, PointField.UINT32, 1)]

            header = Header()
            header.frame_id = "map"
            # marker.color_cloud_ptr = point_cloud2.create_cloud(header, fields, points)
        # print("ParseRoadMarker done, marker id %d, type %d"%(marker.id, marker.type))
        return marker

    def GetNoAccessibleFromFile(self, jsonData):
        polygonArray = MarkerPolygonArray()
        cnt = 0
        print("GetNoAccessibleFromFile num %d"%(len(jsonData['not_accessible'])))
        for i in range(int(len(jsonData['not_accessible']))):
            dataJson = jsonData['not_accessible'][i]
            marker = MarkerPolygon()
            if 'id' in dataJson:
                marker.id = dataJson['id']
            if 'category' in dataJson:
                marker.type = dataJson['category']
            # print("GetNoAccessibleFromFile marker %d"%(marker.id))

            if 'nav_croad_id' in dataJson:
                marker.navgId = dataJson['nav_croad_id']

            if 'lane_id' in dataJson or 'lane_ids' in dataJson:
                if 'lane_id' in dataJson:
                    lane_key = 'lane_id'
                else:
                    lane_key = 'lane_ids'
                # rospy.loginfo("GetNoAccessibleFromFile lane_key: %s, len %d", lane_key, len(dataJson[lane_key]))
                for index in range(len(dataJson[lane_key])):
                    pair = Pair()
                    # print("GetNoAccessibleFromFile lane_id %s"%(dataJson[lane_key][index]))
                    if dataJson[lane_key][index] != '':
                        pair.laneId = int(dataJson[lane_key][index])
                        marker.pairs.append(pair)

            if 'points' in dataJson:
                rect_points = dataJson['points']
                for pid in range(len(rect_points)):
                    pnt = Point();
                    pnt.x = rect_points[pid]['x']
                    pnt.y = rect_points[pid]['y']
                    pnt.z = rect_points[pid]['z']
                    marker.corner_points.append(pnt)

            polygonArray.markers.append(marker)
        self.publisher["PubNonAccessibles"].publish(polygonArray)
        rospy.loginfo("publish topic [/non_accessibles]")

    def GetRoadMarkerFromFile(self, jsonData):
        polygonArray = MarkerPolygonArray()
        cnt = 0
        print("GetRoadMarkerFromFile num %d"%(len(jsonData['roadmarkers'])))
        for i in range(int(len(jsonData['roadmarkers']))):
            laneJson = jsonData['roadmarkers'][i]
            if 'lane_id' in laneJson:
                lane_id = laneJson['lane_id']

            if 'markers' in laneJson:
                for id in range(len(laneJson['markers'])):
                    markerJson = laneJson['markers'][id]
                    cnt += 1
                    marker = self.ParseRoadMarker(markerJson)
                    polygonArray.markers.append(marker)
            else:
                marker = self.ParseRoadMarker(laneJson)
                polygonArray.markers.append(marker)
        self.publisher["PubRoadMarkers"].publish(polygonArray)
        rospy.loginfo("publish topic [/road_markers_display]")

    def GetParkingSpaceFromFile(self, jsonData):
        spaceArray = ParkingSpaceArray()
        cnt = 0
        print("GetParkingSpaceFromFile num %d"%(len(jsonData['parking_space'])))
        for i in range(int(len(jsonData['parking_space']))):
            dataJson = jsonData['parking_space'][i]
            space = ParkingSpace()
            if 'space_id' in dataJson:
                space.id = dataJson['space_id']
            if 'order_type' in dataJson:
                space.orderType = dataJson['order_type']

            if 'lane_id' in dataJson:
                space.laneId = dataJson['lane_id']
            if 'point_id' in dataJson:
                space.pointId = dataJson['point_id']
            if 'type' in dataJson:
                space.type = dataJson['type']
            if 'parkinglot_id' in dataJson:
                space.parkingLotId = dataJson['parkinglot_id']
            if 'side' in dataJson:
                space.side = dataJson['side']
            # rospy.loginfo("GetParkingSpaceFromFile space %d, type %d, side %d, laneId %d, pointId %d", space.id, space.type, space.side, space.laneId, space.pointId)

            # if 'lat' in dataJson:
            #     space.geopoint.latitude = dataJson['lat']
            # if 'lng' in dataJson:
            #     space.geopoint.longitude = dataJson['lng']

            if 'points' in dataJson:
                for id in range(len(dataJson['points'])):
                    pntJson = dataJson['points'][id]
                    orderPnt = Point()
                    if 'x' in pntJson:
                        orderPnt.x = pntJson['x']
                    if 'y' in pntJson:
                        orderPnt.y = pntJson['y']
                    if 'z' in pntJson:
                        orderPnt.z = pntJson['z']
                    space.points.append(orderPnt)
            spaceArray.spaces.append(space)
        self.publisher["PubParkingSpaces"].publish(spaceArray)
        rospy.loginfo("publish topic [/parking_spaces]")

    def GetParkingLotFromFile(self, jsonData):
        lotArray = ParkingLotArray()
        cnt = 0
        print("GetParkingLotFromFile num %d"%(len(jsonData['parking_lot'])))
        for i in range(int(len(jsonData['parking_lot']))):
            dataJson = jsonData['parking_lot'][i]
            lot = ParkingLot()
            if 'id' in dataJson:
                lot.id = dataJson['id']
            if 'name' in dataJson:
                lot.name = dataJson['name']

            if 'nav_croad_id' in dataJson:
                lot.nav_croad_id = dataJson['nav_croad_id']
            if 'point_id' in dataJson:
                lot.pointId1 = dataJson['point_id']
            if 'nav_croad_id2' in dataJson:
                lot.nav_croad_id2 = dataJson['nav_croad_id2']
            if 'point_id2' in dataJson:
                lot.pointId2 = dataJson['point_id2']
            rospy.loginfo("GetParkingLotFromFile parking lot %d(%s), nv1 %d(%d), nv2 %d(%d)", lot.id, lot.name, lot.nav_croad_id, lot.pointId1, lot.nav_croad_id2, lot.pointId2)
            lotArray.lots.append(lot)
        self.publisher["PubParkingLot"].publish(lotArray)
        rospy.loginfo("publish topic [/parking_lots], len %ld", len(lotArray.lots))

    def GetTrafficLightFromFile(self, jsonData):
        lightArray = TrafficLightArray()
        cnt = 0
        print("GetTrafficLightFromFile num %d"%(len(jsonData['traffic_light'])))
        for i in range(int(len(jsonData['traffic_light']))):
            dataJson = jsonData['traffic_light'][i]
            lot = TrafficLight()
            if 'id' in dataJson:
                lot.id = dataJson['id']
            if 'lane_ids' in dataJson and dataJson['lane_ids'] != ' ':
                print(dataJson['lane_ids'])
                if ';' in dataJson['lane_ids']:
                    lanes = dataJson['lane_ids'].split(';')
                    for temp in lanes:
                        lot.laneIds.append(int(temp))
                elif isinstance(dataJson['lane_ids'], int):
                    lot.laneIds.append(int(dataJson['lane_ids']))

            # if 'lat' in dataJson:
            #     lot.geopoint.latitude = dataJson['lat']
            # if 'lng' in dataJson:
            #     lot.geopoint.longitude = dataJson['lng']

            if 'category' in dataJson:
                lot.type = dataJson['category']
            if 'light_num' in dataJson:
                lot.lightNum = dataJson['light_num']
            if 'vps' in dataJson:
                lot.vps = dataJson['vps']
            if 'vpe' in dataJson:
                lot.vpe = dataJson['vpe']
            if 'heading' in dataJson:
                lot.heading = dataJson['heading']
            if 'lights' in dataJson:
                for id in range(len(dataJson['lights'])):
                    if isinstance(dataJson['lights'][id], list):
                        for id2 in range(len(dataJson['lights'][id])):
                            lightstr = dataJson['lights'][id][id2]
                            lot.lights.append(lightstr)
                    else:
                        lightstr = dataJson['lights'][id]
                        rospy.loginfo("GetTrafficLightFromFile 1 lightstr %s", lightstr.encode('utf-8'))
                        lot.lights.append(lightstr)
                    # rospy.loginfo("GetTrafficLightFromFile lights len %d", len(lot.lights))
            if 'points' in dataJson:
                for id in range(len(dataJson['points'])):
                    pointJson = dataJson['points'][id]
                    if isinstance(pointJson, list):
                        for id2 in range(len(pointJson)):
                            point = Point()
                            point.x = float(pointJson[id2]['x'])
                            point.y = float(pointJson[id2]['y'])
                            point.z = float(pointJson[id2]['z'])
                            lot.points.append(point)
                    else:
                        point = Point()
                        point.x = float(pointJson['x'])
                        point.y = float(pointJson['y'])
                        point.z = float(pointJson['z'])
                        lot.points.append(point)
            rospy.loginfo("GetTrafficLightFromFile light %d(%d), vpe %s, vpe %s, heading %f, point %d, lights %d", lot.id, lot.lightNum, lot.vps, lot.vpe, lot.heading, len(lot.points), len(lot.lights))
            lightArray.lights.append(lot)
        self.publisher["PubTrafficLight"].publish(lightArray)
        rospy.loginfo("publish topic [/traffic_lights], len %ld", len(lightArray.lights))

    # def GetDistanceBetweenDimensionPoints(self, point1, point2):
    #     return math.sqrt((point2.x - point1.x) * (point2.x - point1.x) +
    #         (point2.y - point1.y)*(point2.y - point1.y) +
    #         (point2.z - point1.z)*(point2.z - point1.z));

    def GetPointsOfRoadLine(self, points, prevPnt):
        pointId = 1
        pointArray = []
        pointNum = len(points)
        for pid in range(len(points)):
            currentPnt = Point()
            currentPnt.x = points[pid]['x']
            currentPnt.y = points[pid]['y']
            currentPnt.z = points[pid]['z']

            # if prevPnt.x is not 0 and prevPnt.y is not 0 and prevPnt.z is not 0:
                # dist = self.GetDistanceBetweenDimensionPoints(currentPnt, prevPnt)
                # if dist < 0.3:
                #     continue;

            pnt = RoadLinePoint();
            if 'point_id' not in points[pid]:
                pnt.id = pointId
                pointId += 1
            else:
                pnt.id = points[pid]['point_id']
                if len(pointArray) > 0 and pnt.id - 1 > pointArray[len(pointArray) - 1].id:
                    pnt.id = pointArray[len(pointArray) - 1].id + 1

            if 'type' not in points[pid]:
                pnt.type = 0
            else:
                pnt.type = points[pid]['type']
            pnt.position.x = currentPnt.x
            pnt.position.y = currentPnt.y
            pnt.position.z = currentPnt.z

            # if pnt.id == pointNum:
            #     print("point x %f, y %f, z %f", pnt.position.x, pnt.position.y, pnt.position.z)
            pointArray.append(pnt)

            prevPnt.x = pnt.position.x
            prevPnt.y = pnt.position.y
            prevPnt.z = pnt.position.z
        return pointArray, prevPnt

    def CheckFileOrDir(self, filedir):
        json_files = []
        if os.path.isdir(filedir):
            json_files = [f for f in os.listdir(filedir) if f.endswith(".json")]
            json_files = sorted(json_files, key = lambda x: x.rsplit('.', 1)[0])
        else:
            json_files.append(filedir)
            filedir=''
            print("CheckFileOrDir json file name %ld"%(len(json_files)))
        return json_files, filedir

    def ShowAllData(self, filedir):
        pathArray = PathsArray()
        pathNavgArray = NavgRoadArray()
        laneNavgRoadArray = LaneNavgRoadArray()
        laneArray = LanesArray()
        # filedir = self.folder["filedir"] + self.folder["waypoints_folder_name"]
        json_files, filedir = self.CheckFileOrDir(filedir)

        index = 1
        waypointsfile_flag = False
        for file_index in range(len(json_files)):
            time.sleep(0.5)
            with open(filedir + '/' + json_files[file_index]) as json_data:
                rospy.loginfo("ShowAllData filename %s",json_files[file_index])
                jsonData = json.load(json_data)
                if 'waypoints' in jsonData:
                    path=None

                    path = self.ReadChangedWaypointsFile(index, jsonData)
                    waypointsfile_flag = True
                if waypointsfile_flag :
                    path.name = json_files[file_index]
                    pathArray.paths.append(path)
                    index += 1
                    waypointsfile_flag = False

                    if len(pathArray.paths) > 0:
                        self.publisher["PubGlobalPathPoints"].publish(pathArray)
                        rospy.loginfo("publish topic [/global_waypoints_display]")
                    else:
                        rospy.logwarn("pathArray no paths")
                elif 'navgroads' in  jsonData:
                    pathNavgArray = self.ReadNavgRoadsFile(jsonData)

                    if len(pathNavgArray.navgroads) > 0:
                        self.publisher["PubNavgRoadPaths"].publish(pathNavgArray)
                        rospy.loginfo("publish topic [/global_navgroads_display]")
                    else:
                        rospy.logwarn("pathNavgArray no navgroads")
                elif 'lanes_navgroads' in  jsonData:
                    laneNavgRoadArray = self.ReadLanesNavgRoadsFile(jsonData)

                    if len(laneNavgRoadArray.lanenavgroads) > 0:
                        self.publisher["PubLaneNavgRoadPaths"].publish(laneNavgRoadArray)
                        rospy.loginfo("publish topic [/global_lanenavgroads_display]")
                    else:
                        rospy.logwarn("laneNavgRoadArray no lanenavgroads")
                elif 'lanes' in jsonData:
                    self.GetLanes(filedir + '/' + json_files[file_index])
                elif 'roadmarkers' in jsonData:
                    self.GetRoadMarkerFromFile(jsonData)
                elif 'not_accessible' in jsonData:
                    self.GetNoAccessibleFromFile(jsonData)
                elif 'roadlines' in jsonData:
                    self.GetRoadLinesFromFile(jsonData)
                elif 'traffic_light' in jsonData:
                    self.GetTrafficLightFromFile(jsonData)
                elif 'parking_space' in jsonData:
                    self.GetParkingSpaceFromFile(jsonData)
                else:
                    rospy.logwarn('ShowAllData not parse %s', json_files[file_index])


    def ShowRoadLines(self, filedir):
        json_files, filedir = self.CheckFileOrDir(filedir)
        rospy.loginfo("ShowRoadLines Load %s, json file num %ld", filedir, len(json_files))

        for json_file_index in range(len(json_files)):
            with open(filedir + '/' + json_files[json_file_index]) as json_data:
                jsonData = json.load(json_data)
                self.GetRoadLinesFromFile(jsonData)

    def ShowRoadMarker(self, filedir):
        json_files, filedir = self.CheckFileOrDir(filedir)
        print("ShowRoadMarker filedir %s, json file name %ld"%(filedir, len(json_files)))
        for json_file_index in range(len(json_files)):
            with open(filedir + '/' + json_files[json_file_index]) as json_data:
                print(filedir + '/' + json_files[json_file_index])
                jsonData = json.load(json_data)
                if 'roadmarkers' in jsonData:
                    self.GetRoadMarkerFromFile(jsonData)
                elif 'not_accessible' in jsonData:
                    self.GetNoAccessibleFromFile(jsonData)
                else:
                    rospy.logwarn("ShowRoadMarker not marker file")

    def ShowNavgRoads(self, filedir):
        json_files, filedir = self.CheckFileOrDir(filedir)
        print("ShowNavgRoads filedir %s, json file name %ld"%(filedir, len(json_files)))
        for json_file_index in range(len(json_files)):
            with open(filedir + '/' + json_files[json_file_index]) as json_data:
                print(filedir + '/' + json_files[json_file_index])
                jsonData = json.load(json_data)
                if 'lanes_navgroads' in jsonData:
                    laneNavgRoadArray = self.ReadLanesNavgRoadsFile(jsonData)
                    if len(laneNavgRoadArray.lanenavgroads) > 0:
                        self.publisher["PubLaneNavgRoadPaths"].publish(laneNavgRoadArray)
                        rospy.loginfo("publish topic [/global_lanenavgroads_display]")
                elif 'navgroads' in jsonData:
                    pathNavgArray = self.ReadNavgRoadsFile(jsonData)
                    if len(pathNavgArray.navgroads) > 0:
                        self.publisher["PubNavgRoadPaths"].publish(pathNavgArray)
                        rospy.loginfo("publish topic [/global_navgroad_display]")
                elif 'points' in jsonData:
                    self.ReadNavgRoadsFile(jsonData)
                else:
                    rospy.logwarn("ShowNavgRoads not marker file")

    def ShowParkingSpace(self, filedir):
        json_files, filedir = self.CheckFileOrDir(filedir)
        print("ShowParkingSpace filedir %s, json file name %ld"%(filedir, len(json_files)))
        for json_file_index in range(len(json_files)):
            with open(filedir + '/' + json_files[json_file_index]) as json_data:
                print(filedir + '/' + json_files[json_file_index])
                jsonData = json.load(json_data)
                if 'parking_space' in jsonData:
                    self.GetParkingSpaceFromFile(jsonData)
                elif 'parking_lot' in jsonData:
                    self.GetParkingLotFromFile(jsonData)
                else:
                    rospy.logwarn("ShowParkingSpace not parking related file")
    def ShowTrafficLight(self, filedir):
        json_files, filedir = self.CheckFileOrDir(filedir)
        print("ShowTrafficLight filedir %s, json file name %ld"%(filedir, len(json_files)))
        for json_file_index in range(len(json_files)):
            with open(filedir + '/' + json_files[json_file_index]) as json_data:
                print(filedir + '/' + json_files[json_file_index])
                jsonData = json.load(json_data)
                if 'traffic_light' in jsonData:
                    self.GetTrafficLightFromFile(jsonData)
                else:
                    rospy.logwarn("ShowTrafficLight not parking related file")

    def ShowGlobalPath(self, filedir):
        pathArray = PathsArray()
        json_files, filedir = self.CheckFileOrDir(filedir)

        index = 1
        waypointsfile_flag = False
        for file_index in range(len(json_files)):
            rospy.loginfo("ShowGlobalPath filename %s",json_files[file_index])
            with open(filedir + '/' + json_files[file_index]) as json_data:
                jsonData = json.load(json_data)
                path = Paths()
                waypointsfile_flag = False
                if 'waypoints' in jsonData:
                    waypointsfile_flag = True
                    # if os.path.exists(self.folder["raw_data_path"]):
                    #     rospy.logwarn("ShowAllData:: splited file exist")
                    #     with open(self.folder["raw_data_path"]) as json_data:
                    #         jsonData = json.load(json_data)
                    #     path = self.ReadChangedWaypointsFile(index, jsonData)
                    # else:
                    path = self.ReadChangedWaypointsFile(index, jsonData)
                # elif 'global_path' in jsonData:
                #     waypointsfile_flag = True
                #     path = self.ReadWaypointJsonFile(index, jsonData)

                if waypointsfile_flag:
                    path.name = json_files[file_index]
                    pathArray.paths.append(path)
                    index += 1

        if len(pathArray.paths) > 0:
            self.publisher["PubGlobalPathPoints"].publish(pathArray)
            rospy.loginfo("publish topic [/global_waypoints_display]")

if __name__ == '__main__' :
    try:
        rospy.loginfo('main')
        rospy.init_node('load_file_node', anonymous=True)
        rospack = rospkg.RosPack()
        pkgPath = rospack.get_path('route_mission_handler')

        folder = {}
        route = rospy.get_param('~route', None)
        folder["waypoints_folder_name"] = rospy.get_param('~waypoints_folder_path', None)
        folder["roadline_folder_name"] = rospy.get_param('~roadline_folder_path', None)
        folder["marker_folder_name"] = rospy.get_param('~marker_folder_path', None)

        publisher = {}
        navgroad_display_pub = rospy.get_param('~navgroad_display', None)
        lanenavgroad_display_pub = rospy.get_param('~lanenavgroad_display', None)

        waypoints_display_pub = rospy.get_param('~waypoints_display', None)
        roadline_display_pub = rospy.get_param('~roadline_display', None)
        roadmarker_display_pub = rospy.get_param('~roadmarker_display', None)
        intersections_display_pub = rospy.get_param('~intersections_display', None)

        publisher["PubLaneNavgRoadPaths"] = rospy.Publisher(lanenavgroad_display_pub, LaneNavgRoadArray, queue_size=10)
        publisher["PubNavgRoadPaths"] = rospy.Publisher(navgroad_display_pub, NavgRoadArray, queue_size=10)
        publisher["PubGlobalPathPoints"] = rospy.Publisher(waypoints_display_pub, PathsArray, queue_size=10)
        publisher["PubRoadLine"] = rospy.Publisher(roadline_display_pub, RoadLineArray, queue_size=10)
        publisher["PubRoadMarkers"] = rospy.Publisher(roadmarker_display_pub, MarkerPolygonArray, queue_size=10)
        publisher["PubLanes"] = rospy.Publisher('/laneinfo_fromfile', LanesArray, queue_size=10)
        publisher["PubNonAccessibles"] = rospy.Publisher('/non_accessibles', MarkerPolygonArray, queue_size=10)
        publisher["PubParkingSpaces"] = rospy.Publisher('/parking_spaces', ParkingSpaceArray, queue_size=10)
        publisher["PubParkingLot"] = rospy.Publisher('/parking_lots', ParkingLotArray, queue_size=10)
        publisher["PubTrafficLight"] = rospy.Publisher('/traffic_lights',TrafficLightArray, queue_size=10)

        loadCompletedPub = rospy.Publisher('/load_completed', Bool, queue_size=10)

        folder["filedir"] = pkgPath + '/data/' + route + '/'

        filemanager = FileManager(folder, publisher)
        filemanager.ShowAllData(folder['filedir'])
        rospy.loginfo("ShowAllData finish!")

        doneMsg = Bool()
        loadCompletedPub.publish(doneMsg)
        rospy.loginfo("publish load completey!")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
