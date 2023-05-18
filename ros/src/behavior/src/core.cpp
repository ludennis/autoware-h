#include <behavior/core.h>
#include <behavior/lane_following.h>
#include <stdint.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <trace/utils.h>

using hash_t = size_t;
constexpr hash_t prime = 0x100000001B3ull;
constexpr hash_t basis = 0xCBF29CE484222325ull;
static const float HALF = 0.5f;
static const float RESOLUTION = 1.0f;
static const float THRESHOLD_SAFETY_FACTOR = 1.0f;

namespace Behavior
{
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

    static const float FREQUENCE = 100.0f;
    static const float ACC_LIMIT = 5.0f;

    template<typename KeyType>
    std::pair<KeyType,SpeedCmd> MapGetMin( const std::map<KeyType,SpeedCmd>& map ) {
        using pairtype=std::pair<KeyType,SpeedCmd>;
        return *std::min_element(map.begin(), map.end(), []
            (const pairtype & p1, const pairtype & p2) {
            return p1.second.acceleration < p2.second.acceleration;
        });
    }

    template <typename T>
    static inline T Clamp(const T & value, T bottom, T top)
    {
        return std::max(bottom, std::min(top, value));
    }

    static inline float GetPrincipalAngle(const float & angle)
    {
        float angleMod = std::fmod(angle, 2.0f * M_PI);
        if (std::signbit(angleMod))
            angleMod += 2.0f * M_PI;
        return angleMod;
    }

    static inline bool CheckPtNanorInfinite(const float x, const float y)
    {
        bool check = false;
        if (std::isnan(x) || std::isnan(y) || !std::isfinite(x) || !std::isfinite(y))
            check = true;

        return check;
    }

    static inline float PrincipleAngleMinusPi(const float angle)
  {
      return std::atan2(std::sin(angle), std::cos(angle));
  }

  static inline float SupplementaryAngle(const float angle)
  {
      return PrincipleAngleMinusPi(M_PI - angle);
  }

    int Core::ClosestWaypoint(
        float nowX,
        float nowY,
        const std::vector<float> & GlobalX,
        const std::vector<float> & GlobalY,
        int lastClosestWaypoint,
        int searchingRange)
    {
        float closestDist = 100000; //large number
        int closestWaypoint = 0;
        int upperBound;
        int lowerBound;
        int lowerMidBound;
        int upperMidBound;
        if (lastClosestWaypoint + searchingRange >= GlobalX.size())
        {
            lowerBound = lastClosestWaypoint - searchingRange;
            upperBound = (lastClosestWaypoint + searchingRange) %
                GlobalX.size();
            lowerMidBound = GlobalX.size() - 1;
            upperMidBound = 0;
        }
        else if (lastClosestWaypoint - searchingRange <= 0)
        {
            lowerBound = (lastClosestWaypoint - searchingRange) %
                GlobalX.size() + GlobalX.size();
            upperBound = lastClosestWaypoint + searchingRange;
            lowerMidBound = GlobalX.size() - 1;
            upperMidBound = 0;
        }
        else
        {
            lowerBound = lastClosestWaypoint - searchingRange;
            upperBound = lastClosestWaypoint + searchingRange;
            lowerMidBound = lastClosestWaypoint;
            upperMidBound = lastClosestWaypoint + 1;
        }

        for (int i = 0; i < GlobalX.size(); i++)
        {
            float mapX = GlobalX[i];
            float mapY = GlobalY[i];
            float distance = std::hypot(mapX - nowX, mapY -nowY);

            if (distance < closestDist &&
                ((i >= lowerBound && i <= lowerMidBound) ||
                (i >= upperMidBound && i <= upperBound)))
            {
                closestDist = distance;
                closestWaypoint = i;
            }
        }
        if (closestDist ==  100000)
        {
            closestWaypoint = lastClosestWaypoint;
        }
        return closestWaypoint;
    }

    void Core::LocalPathROI(
        const itri_msgs::WaypointArray & LocalWayPointsXY,
        const std::vector<float> & LocalX,
        const std::vector<float> & LocalY,
        const VehPosXY & vehPosXY,
        itri_msgs::WaypointArray & AccRoiXY)
    {
        int closestIndex = ClosestWaypoint(
            vehPosXY.x,
            vehPosXY.y,
            LocalX,
            LocalY,
            0,
            LocalX.size());

        int waypointSize =
            static_cast<int>(LocalWayPointsXY.waypoints.size()) - closestIndex;

        AccRoiXY.waypoints.resize(waypointSize / 15);

        for (int i = 0;
            i < (waypointSize / 15); ++i)
            AccRoiXY.waypoints[i] = LocalWayPointsXY.waypoints[closestIndex + 15 * i];
    }

    int Core::ClosestWaypoint(
        float nowX,
        float nowY,
        const std::vector<float> & GlobalX,
        const std::vector<float> & GlobalY,
        int lastClosestWaypoint,
        int searchingRange,
        const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr & kdtree)
    {
        pcl::PointXYZ searchPoint;
        searchPoint.x = nowX;
        searchPoint.y = nowY;
        searchPoint.z = 0.0f;
        std::vector<int> idxRadiusSearch;
        std::vector<float> radiusSquaredDistance;

        if (!kdtree)
            return 0;

        kdtree->radiusSearch(
            searchPoint, 10.0, idxRadiusSearch, radiusSquaredDistance);

        if (idxRadiusSearch.size() > 0)
        {
            int closestIdxOfSearch = 0;
            for (int i = 0; i < idxRadiusSearch.size() &&
                std::abs(idxRadiusSearch[i] - lastClosestWaypoint) > 10; i ++)
            {
                closestIdxOfSearch  = i + 1;
            }


            if (closestIdxOfSearch < idxRadiusSearch.size())
            {
                return idxRadiusSearch[closestIdxOfSearch];
            }
            else
            {
                std::vector<int> idxNKNSearch(1);
                std::vector<float> nKNSquaredDistance(1);
                kdtree->nearestKSearch(
                    searchPoint, 1, idxNKNSearch, nKNSquaredDistance);
                return idxNKNSearch[0];
            }
        }
        else
        {
            std::vector<int> idxNKNSearch(1);
            std::vector<float> nKNSquaredDistance(1);
            kdtree->nearestKSearch(
                searchPoint, 1, idxNKNSearch, nKNSquaredDistance);
            return idxNKNSearch[0];
        }
    }

    int Core::NextWaypoint(
        float nowX,
        float nowY,
        float Theta,
        const std::vector<float> & GlobalX,
        const std::vector<float> & GlobalY,
        int lastClosestWaypoint,
        int searchingRange,
        const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr & kdtree)
    {
        // std::vector<float> x_tmp(GlobalX.begin()+std::max(lastClosestWaypoint-80,0),
        //     GlobalX.begin()+std::min(lastClosestWaypoint+80,static_cast<int>(GlobalX.size())));
        // std::vector<float> y_tmp(GlobalY.begin()+std::max(lastClosestWaypoint-80,0),
        //     GlobalY.begin()+std::min(lastClosestWaypoint+80,static_cast<int>(GlobalY.size())));

        int closestWaypoint = ClosestWaypoint(
            nowX,
            nowY,
            GlobalX,
            GlobalY,
            lastClosestWaypoint,
            searchingRange, kdtree);
        // closestWaypoint = lastClosestWaypoint;

        float mapX = GlobalX[closestWaypoint];
        float mapY = GlobalY[closestWaypoint];

        float heading = std::atan2((mapY - nowY), (mapX - nowX));
        float angle = std::abs(Theta-heading);

        float distance = std::hypot(mapX - nowX, mapY -nowY);

        if (angle > M_PI / 4 || distance == 0)
        {
            closestWaypoint++;
        }
        closestWaypoint = std::max(0,
            std::min(closestWaypoint,static_cast<int>(GlobalX.size() - 1)));
        return closestWaypoint;
    }

    void Core::TransformObjectXY(
        const tf::StampedTransform & transform,
        ObjectXY & objPoints,
        const itri_msgs::Ars40xObject& object)
    {
        objPoints.id = 400 + object.id;
        float positionX = object.pose.position.x;
        float positionY = object.pose.position.y;
        tf::Point pt(positionX, positionY, 0);
        tf::Point ptWorld = transform * pt;

        objPoints.centerX = ptWorld.x();
        objPoints.centerY = ptWorld.y();

        objPoints.relativeSpeedX = object.velocity.linear.x;
        objPoints.relativeSpeedY = object.velocity.linear.y;
    }

    void Core::TransformObjectXY(
        const tf::StampedTransform & transform,
        ObjectXY & objPoints,
        const itri_msgs::DetectedObject& object)
    {
        objPoints.id = object.id;
        float positionX = object.pose.position.x;
        float positionY = object.pose.position.y;
        tf::Point pt(positionX, positionY, 0);
        tf::Point ptWorld = transform * pt;

        objPoints.centerX = ptWorld.x();
        objPoints.centerY = ptWorld.y();

        objPoints.state = object.behavior_state;
        objPoints.relativeSpeedX = object.velocity.linear.x;
        objPoints.relativeSpeedY = object.velocity.linear.y;
        if (object.convex_hull.polygon.points.size() > 0)
        {
            for (size_t j = 0; j < object.convex_hull.polygon.points.size(); ++j)
            {
                tf::Point convexHullPoint(
                    object.convex_hull.polygon.points[j].x,
                    object.convex_hull.polygon.points[j].y,
                    object.convex_hull.polygon.points[j].z);
                tf::Point worldPoint = transform * convexHullPoint;
                    objPoints.x.push_back(worldPoint.x());
                    objPoints.y.push_back(worldPoint.y());
                    objPoints.z.push_back(worldPoint.z());
            }
        }
    }

    void Core::FrenetTransform(
        float localX,
        float localY,
        float Theta,
        const std::vector<float> & globalX,
        const std::vector<float> & globalY,
        float & globalS,
        float & globalD,
        int lastClosestWaypoint,
        int searchingRange,
        const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr & kdtree)
    {
        std::vector<float> glovalPos =
            GetFrenet(
            localX,
            localY,
            Theta,
            globalX,
            globalY,
            lastClosestWaypoint,
            searchingRange, kdtree);
        globalS = glovalPos[0];
        globalD = glovalPos[1];
    }

    void Core::FrenetTransform(
        std::vector<float> & localX,
        std::vector<float> & localY,
        std::vector<float> & Theta,
        const std::vector<float> & globalX,
        const std::vector<float> & globalY,
        std::vector<float> & globalS,
        std::vector<float> & globalD,
        int lastClosestWaypoint,
        int searchingRange,
        const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr & kdtree)
    {
        std::vector<float> GlobalPoints;
        for (int i = 0; i < localX.size(); i++)
        {
            GlobalPoints = GetFrenet(
                localX[i],
                localY[i],
                Theta[i],
                globalX,
                globalY,
                lastClosestWaypoint,
                searchingRange, kdtree);
            globalS.push_back(GlobalPoints[0]);
            globalD.push_back(GlobalPoints[1]);
        }
    }

    void Core::FrenetTransform(
        std::vector<ObjectXY> & objListXY,
        const std::vector<float> & globalX,
        const std::vector<float> & globalY,
        const std::vector<float> & globalHeading,
        const VehPosXY & vehPosXY,
        std::vector<ObjectSD> & objListSD,
        int lastClosestWaypoint,
        int searchingRange,
        const float steeringAngle,
        const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr & kdtree)
    {
        for (int i = 0; i < objListXY.size(); i++)
        {
            ObjectSD points;
            points.relativeSpeedS = objListXY[i].relativeSpeedX;
            points.relativeSpeedD = objListXY[i].relativeSpeedY;
            points.id = objListXY[i].id;
            points.state = objListXY[i].state;
            bool checkPtNanInfinite = false;
            std::vector<float> objPoints;
            std::vector<float> objCenterPoints;
            checkPtNanInfinite =
                CheckPtNanorInfinite(objListXY[i].centerX, objListXY[i].centerY);
            if (!checkPtNanInfinite)
            {
                objCenterPoints = GetFrenet(
                    objListXY[i].centerX,
                    objListXY[i].centerY,
                    0,
                    globalX,
                    globalY,
                    lastClosestWaypoint,
                    searchingRange, kdtree);
            }
            else
            {
                objCenterPoints = GetFrenet(
                    objListXY[i].x[0],
                    objListXY[i].y[0],
                    0,
                    globalX,
                    globalY,
                    lastClosestWaypoint,
                    searchingRange, kdtree);
            }
            points.centerS = objCenterPoints[0];
            points.centerD = objCenterPoints[1];

            for (int j = 0; j < objListXY[i].x.size(); j++)
            {
                checkPtNanInfinite =
                    CheckPtNanorInfinite(objListXY[i].x[j], objListXY[i].y[j]);
                if (!checkPtNanInfinite)
                {
                    objPoints = GetFrenet(
                        static_cast<float>(objListXY[i].x[j]),
                        static_cast<float>(objListXY[i].y[j]),
                        0,
                        globalX,
                        globalY,
                        lastClosestWaypoint,
                        searchingRange, kdtree);
                    points.s.push_back(objPoints[0]);
                    points.d.push_back(objPoints[1]);
                    ToSpeedXY(vehPosXY, objListXY[i], steeringAngle);
                    ToSpeedSD(
                        objListXY[i],
                        points,
                        globalX,
                        globalY,
                        globalHeading,
                        lastClosestWaypoint,
                        searchingRange, kdtree);
                }
            }
            objListSD.push_back(points);
        }
    }

    void Core::BumperDetect(
        bool & bumper, int & currentBumper, const int nextBumper,
        const int lastClosestWaypoint, const float maxSpeed, const float vehicleS)
    {
        if ((nextBumper - lastClosestWaypoint) > -5.0f &&
            ((nextBumper - lastClosestWaypoint) < std::max(20.0f, std::pow(maxSpeed, 2.0f)/(2.0f*1.0f))))
        {
            bumper = true;
        }

        if (static_cast<float>(vehicleS - currentBumper) > 0.0f || vehicleS < 0.0f)
        {
            currentBumper = nextBumper;
        }
    }

    void Core::CurveDetect(
        bool & enterCurve, const float vehiclePositionS,
        const std::vector<float> & globalPathS,
        const std::vector<float> & globalPathCurvature,
        const std::vector<int> & indexStartCurve,
        const std::vector<int> & indexEndCurve,
        const float maxSpeed,
        float & curveSpeed)
    {
        LaneFollowing LF;
        if (indexEndCurve.size() > 0)
        {
             int curveNum = LF.NextCurveIndex(vehiclePositionS, indexEndCurve, globalPathS);
            enterCurve = LF.CurveAvoid(
                vehiclePositionS, globalPathS, globalPathCurvature,
                indexStartCurve, indexEndCurve, curveNum, maxSpeed, curveSpeed);
            if(!enterCurve) curveSpeed = -1.0f;
        }
        else
        {
            enterCurve = false;
        }
    }

    void Core::ToSpeedXY(
        const VehPosXY & vehPosXY,
        ObjectXY & objectXY,
        const float steeringAngle)
    {
        const float wheelBase = 2614.0f;
        const float distanceToRearWheel = 1190.0f;
        float slipAngle = std::atan(
            (distanceToRearWheel * std::tan(steeringAngle)) / wheelBase);

        float orthogonalDeg = GetPrincipalAngle(vehPosXY.heading + M_PI_2);

        float projectionX =
            objectXY.relativeSpeedX * cos(vehPosXY.heading) +
            objectXY.relativeSpeedY * cos(orthogonalDeg);
        float projectionY =
            objectXY.relativeSpeedX * sin(vehPosXY.heading) +
            objectXY.relativeSpeedY * sin(orthogonalDeg);
        objectXY.speedX =
            vehPosXY.speed * cos(vehPosXY.heading + slipAngle) + projectionX;
        objectXY.speedY =
            vehPosXY.speed * sin(vehPosXY.heading + slipAngle) + projectionY;
    }

    void Core::ToSpeedSD(
        const ObjectXY & objectXY,
        ObjectSD & objListSD,
        const std::vector<float> & globalX,
        const std::vector<float> & globalY,
        const std::vector<float> & globalHeading,
        int lastClosestWaypoint,
        int searchingRange,
        const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr & kdtree)
    {
        int nextWayPoint = NextWaypoint(
            objectXY.x[0],
            objectXY.y[0],
            0.0f,
            globalX,
            globalY,
            lastClosestWaypoint,
            100.0f, kdtree);

        int prevWayPoint;
        if (nextWayPoint == 0)
        {
            prevWayPoint  = 0;
            nextWayPoint  = 1;
        }
        else
        {
            prevWayPoint = nextWayPoint-1;
        }

        float globalVectorX = globalX[nextWayPoint]-globalX[prevWayPoint];
        float globalVectorY = globalY[nextWayPoint]-globalY[prevWayPoint];
        float localVectorX = objectXY.x[0] - globalX[prevWayPoint];
        float localVectorY = objectXY.y[0] - globalY[prevWayPoint];

        float projNorm =
            (localVectorX * globalVectorX + localVectorY * globalVectorY) /
            (globalVectorX * globalVectorX + globalVectorY * globalVectorY);

        float mapHeading;
        if (projNorm >= 0)
        {
            mapHeading = globalHeading[nextWayPoint - 1];
        }
        else
        {
            if (nextWayPoint - 2 < 0)
            {
                mapHeading = globalHeading[0];
            }
            else
            {
                mapHeading = globalHeading[nextWayPoint - 2];
            }
        }
        float objXYHeading = std::atan2(objectXY.speedY, objectXY.speedX);
        objXYHeading = GetPrincipalAngle(objXYHeading);
        float deltaTheta = objXYHeading - mapHeading;
        deltaTheta = GetPrincipalAngle(deltaTheta);
        float magnitude = std::hypot(objectXY.speedX, objectXY.speedY);

        objListSD.speedS = magnitude * cos(deltaTheta);
        objListSD.speedD = magnitude * sin(deltaTheta);
    }

    int Core::NextWaypoint(
        float nowX,
        float nowY,
        float Theta,
        const std::vector<float> & GlobalX,
        const std::vector<float> & GlobalY,
        int lastClosestWaypoint,
        int searchingRange)
    {
        int closestWaypoint = ClosestWaypoint(
            nowX,
            nowY,
            GlobalX,
            GlobalY,
            lastClosestWaypoint,
            searchingRange);

        float mapX = GlobalX[closestWaypoint];
        float mapY = GlobalY[closestWaypoint];

        float heading = std::atan2((mapY - nowY), (mapX - nowX));
        float angle = std::abs(Theta-heading);

        float distance = std::hypot(mapX - nowX, mapY -nowY);

        if (angle > M_PI / 4 || distance == 0)
        {
            closestWaypoint++;
        }
        closestWaypoint = std::max(0,
            std::min(closestWaypoint,static_cast<int>(GlobalX.size() - 1)));
        return closestWaypoint;
    }

    std::vector<float> Core::GetFrenet(
        float localX,
        float localY,
        float Theta,
        const std::vector<float> & globalX,
        const std::vector<float> & globalY,
        int lastClosestWaypoint,
        int searchingRange)
    {
        int nextWayPoint = NextWaypoint(
            localX,
            localY,
            Theta,
            globalX,
            globalY,
            lastClosestWaypoint,
            searchingRange);
        int prevWayPoint;

        if (nextWayPoint == 0)
        {
            prevWayPoint  = 0;
            nextWayPoint  = 1;
        }
        else
        {
            prevWayPoint = nextWayPoint-1;
        }

        float globalVectorX = globalX[nextWayPoint]-globalX[prevWayPoint];
        float globalVectorY = globalY[nextWayPoint]-globalY[prevWayPoint];
        float localVectorX = localX - globalX[prevWayPoint];
        float localVectorY = localY - globalY[prevWayPoint];

        // find the projection of localVector onto globalVector
        float projNorm =
            (localVectorX * globalVectorX + localVectorY * globalVectorY) /
            (globalVectorX * globalVectorX + globalVectorY * globalVectorY);
        float projX = projNorm * globalVectorX;
        float projY = projNorm * globalVectorY;

        float frenetD = std::hypot(projX - localVectorX, projY - localVectorY);

        const float dotProduct = globalVectorX * localVectorY - globalVectorY * localVectorX;

        if (dotProduct < 0)
        {
            frenetD *= -1;
        }

        // calculate s value
        float frenetS = static_cast<float>(prevWayPoint);
        for (int i = 0; i < prevWayPoint; i++)
        {
            frenetS +=
                std::hypot(globalX[i+1] - globalX[i], globalY[i+1] - globalY[i]);
        }

        frenetS +=  std::copysign(std::hypot(projX,projY), projNorm);
        return {frenetS,frenetD};
    }

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    std::vector<float> Core::GetFrenet(
        float localX,
        float localY,
        float Theta,
        const std::vector<float> & globalX,
        const std::vector<float> & globalY,
        int lastClosestWaypoint,
        int searchingRange,
        const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr & kdtree)
    {
        int nextWayPoint = NextWaypoint(
            localX,
            localY,
            Theta,
            globalX,
            globalY,
            lastClosestWaypoint,
            searchingRange, kdtree);
        int prevWayPoint;

        if (nextWayPoint == 0)
        {
            prevWayPoint  = 0;
            nextWayPoint  = 1;
        }
        else
        {
            prevWayPoint = nextWayPoint-1;
        }

        float globalVectorX = globalX[nextWayPoint]-globalX[prevWayPoint];
        float globalVectorY = globalY[nextWayPoint]-globalY[prevWayPoint];
        float localVectorX = localX - globalX[prevWayPoint];
        float localVectorY = localY - globalY[prevWayPoint];

        // find the projection of localVector onto globalVector
        float projNorm =
            (localVectorX * globalVectorX + localVectorY * globalVectorY) /
            (globalVectorX * globalVectorX + globalVectorY * globalVectorY);
        float projX = projNorm * globalVectorX;
        float projY = projNorm * globalVectorY;

        float frenetD = std::hypot(projX - localVectorX, projY - localVectorY);

        const float dotProduct = globalVectorX * localVectorY - globalVectorY * localVectorX;

        if (dotProduct < 0)
        {
            frenetD *= -1;
        }

        // calculate s value
        float frenetS = static_cast<float>(prevWayPoint);
        // for (int i = 0; i < prevWayPoint; i++)
        // {
        //     frenetS +=
        //         std::hypot(globalX[i+1] - globalX[i], globalY[i+1] - globalY[i]);
        // }

        frenetS +=  std::copysign(std::hypot(projX,projY), projNorm);
        return {frenetS,frenetD};
    }

    ObjRectBound Core::ObjBound(const ObjectSD & objList)
    {
        ObjRectBound objBound;
        std::vector<float> vectorS;
        std::vector<float> vectorD;
        for (int i = 0; i < objList.s.size(); i++)
        {
            vectorS.push_back(objList.s[i]);
            vectorD.push_back(objList.d[i]);
        }
        auto tmpSBound =
            std::minmax_element(std::begin(vectorS), std::end(vectorS));
        auto tmpDBound =
            std::minmax_element(std::begin(vectorD), std::end(vectorD));

        objBound.maxS = *tmpSBound.second;
        objBound.minS = *tmpSBound.first;
        objBound.centerS = objList.centerS;
        objBound.centerD = objList.centerD;
        objBound.rightBound = *tmpDBound.first;
        objBound.leftBound = *tmpDBound.second;
        return objBound;
    }

    std::vector<float> Core::LaneOccupied(
        float vehPosD,
        float roadWidth,
        float rightBound,
        float leftBound)
    {
        std::vector<float> laneOccupiedspace = {0.0f, 0.0f, 0.0f};
        if (roadWidth <= 0)
        {
            return laneOccupiedspace;
        }
        int laneBias = floor((floor(vehPosD / (roadWidth / 2)) + 1) / 2);
        for(unsigned int i = 1; i < 4; i++)
        {
            float laneLinePose = (laneBias + 1.5 - i) * roadWidth;
            float leftBoundOccupied = leftBound - laneLinePose;
            float rightBoundOccupied = rightBound - laneLinePose - roadWidth;
            if (leftBoundOccupied * rightBoundOccupied < 0)
            {
                leftBoundOccupied = std::min(
                    std::abs(leftBoundOccupied), roadWidth);
                rightBoundOccupied = std::min(std::abs(
                    rightBoundOccupied), roadWidth);
                laneOccupiedspace[i-1] =
                    leftBoundOccupied + rightBoundOccupied - roadWidth;
            }
        }
        return laneOccupiedspace;
    }

    float Core::ObjDistance(
        float vehPosS,
        float objFront,
        float objBack,
        float frontSafeDist,
        float backSafeDist)
    {
        float laneOccupiedDistance = 100;
        if (objFront - (vehPosS + frontSafeDist) >= 0)
        {
            if (objBack - vehPosS >= 0)
            {
                laneOccupiedDistance = objBack - vehPosS;
            }
            else
            {
                laneOccupiedDistance = 0;
            }
        }
        else
        {
            if (objBack - vehPosS >= 0)
            {
                laneOccupiedDistance = objBack - vehPosS;
            }
            if (objFront - vehPosS < 0 && objFront -(vehPosS - backSafeDist) > 0)
            {
                laneOccupiedDistance = objFront - vehPosS;
            }
        }
        return laneOccupiedDistance;
    }

    std::vector<bool> Core::LaneSafeHint(
        float roadWidth,
        std::vector<float> laneOccupiedspace,
        ObjectHints objectHints)
    {
        std::vector<bool> safeHint = {false, false, false};
        float freeSpace = 2.5;
        if (objectHints.leftSideIsSafe  == true &&
            laneOccupiedspace[0] < roadWidth - freeSpace )
        {
            safeHint[0] = true;
        }
        if (laneOccupiedspace[1] < roadWidth - freeSpace)
        {
            safeHint[1] = true;
        }
        if (objectHints.rightSideIsSafe == true &&
            laneOccupiedspace[2] < roadWidth - freeSpace )
        {
            safeHint[2] = true;
        }
        return safeHint;
    }

    void Core::SetBehaviorOutput(
        itri_msgs::plan & behaviorPlanOut,
        BehaviorOutput & behaviorOutput)
    {
        behaviorPlanOut.hint = behaviorOutput.hint;
        behaviorPlanOut.num_level = behaviorOutput.numLevel;
        behaviorPlanOut.dev = behaviorOutput.dev;
        behaviorPlanOut.bias = behaviorOutput.bias;
        behaviorPlanOut.terminal_speed = behaviorOutput.terminal_speed;
        behaviorPlanOut.terminal_distance =
            behaviorOutput.terminal_distance;
        behaviorPlanOut.update_num_level =
            behaviorOutput.update_num_level;
        behaviorPlanOut.update_dev = behaviorOutput.update_dev;
        behaviorPlanOut.update_bias = behaviorOutput.update_bias;
        behaviorPlanOut.update_terminal_speed =
            behaviorOutput.update_terminal_speed;
        behaviorPlanOut.update_terminal_distance =
            behaviorOutput.update_terminal_distance;
        behaviorPlanOut.parking_start = behaviorOutput.parking_start;
    }

    void Core::BehaviorStateOutput(
        const State & behavior,
        const bool obstacleHazard,
        itri_msgs::BehaviorState & behaviorState)
    {
        if (behavior == State::LANE_FOLLOW && obstacleHazard)
        {
            behaviorState.behavior_state =static_cast<int>(State::ACC);
        }
        else
        {
            behaviorState.behavior_state = static_cast<int>(behavior);
        }
    }

    ObjectHints Core::ObjectInLaneDetector(
        int dist2Map,
        float vehPosS,
        float vehPosD,
        float vehSpeed,
        const float rollInFactor,
        const std::vector<float> & roadWidth,
        const std::vector<ObjectSD> & objList)
    {
        float frontSafeDist = vehSpeed * rollInFactor + 15;
        float backSafeDist = 10;

        ObjectHints objectHints;
        objectHints.frontObjDistance = 100.0;
        objectHints.backObjDistance = 100.0;
        objectHints.leftSideIsSafe = true;
        objectHints.rightSideIsSafe = true;

        for (unsigned int i = 0; i < objList.size(); i++)
        {
            ObjRectBound objBoundary = ObjBound(objList[i]);
            std::vector<float> laneOccupiedspace = LaneOccupied(
                vehPosD, roadWidth[dist2Map],  objBoundary.rightBound,
                objBoundary.leftBound);
            std::vector<bool> safeHint = LaneSafeHint(roadWidth[dist2Map],
                laneOccupiedspace, objectHints);
            float laneOccupiedDistance = ObjDistance(vehPosS, objBoundary.maxS,
                objBoundary.minS, frontSafeDist, backSafeDist);
            if (!safeHint[0] && laneOccupiedDistance != 100)
            {
                objectHints.leftSideIsSafe = false;
            }
            if (!safeHint[2] && laneOccupiedDistance != 100)
            {
                objectHints.rightSideIsSafe = false;
            }
            if (!safeHint[1])
            {
                if (laneOccupiedDistance >= 0 &&
                    laneOccupiedDistance < objectHints.frontObjDistance)
                {
                    objectHints.frontObjDistance = laneOccupiedDistance;
                }
                if (laneOccupiedDistance < 0 &&
                    laneOccupiedDistance > -objectHints.backObjDistance)
                {
                    objectHints.backObjDistance = -laneOccupiedDistance;
                }
            }
        }
        return objectHints;
    }

    float Core::MedianFilter(std::vector<float> & input, const int window)
    {
        float median;
        int windowSize;
        std::sort(input.begin(), input.end());
        if (window % 2 == 1)
        {
            windowSize = (window + 1) / 2 - 1;
            median = input[windowSize];
        }
        else
        {
            windowSize = window / 2;
            median = (input[windowSize] + input[windowSize - 1]) / 2;
        }
        return median;
    }

    void Core::GetMedianFilterVector(
        const std::vector<float> & input,
        std::vector<float> & output,
        const int window)
    {
        std::vector<float> sortVector;
        int begin;
        int end;
        if (window % 2 == 1)
        {
            begin = -(window - 1) / 2;
            end = (window - 1) / 2 + 1;
        }
        else
        {
            begin = -window / 2;
            end = window / 2;
        }
        for (int i = 0; i < input.size(); ++i)
        {
            for (int j = i + begin; j < i + end; ++j)
            {
                if (std::signbit(j) || j > (input.size() - 1))
                {
                    sortVector.push_back(0);
                }
                else
                {
                    sortVector.push_back(input[j]);
                }
            }
            float medianNumber = MedianFilter(sortVector, window);
            output.push_back(medianNumber);
            sortVector.clear();
        }
    }

    float Core::GetRadOfCurvature(
        const float & prevX, const float & prevY,
        const float & targetX, const float & targetY,
        const float & nextX, const float & nextY)
    {
        const double minConst = 0.000000005;
        float sideA = std::hypot(targetX - prevX, targetY - prevY);
        float sideB = std::hypot(nextX - prevX, nextY - prevY);
        float sideC = std::hypot(nextX - targetX, nextY - targetY);
        double s = (sideA + sideB + sideC) / 2;
        if ((s - sideA == 0) || (s - sideB == 0) || (s - sideC == 0))
        {
            s += minConst;
        }
        float triangleArea =
            std::sqrt(std::fabs(s * (s - sideA) * (s - sideB) * (s - sideC)));
        float Radius = (sideA * sideB * sideC) / (4 * triangleArea);
        return Radius;
    }
    void Core::SpeedProtection(
        float & speed, float & targetAcc,float & targetSpeed,
        const double durationTime, const float CmdSpeed, const float CmdAcce)
    {
        targetSpeed = CmdSpeed;
        targetAcc = CmdAcce * durationTime;
        if(std::signbit((speed - targetSpeed) * CmdAcce))
        {
            speed += targetAcc;
            speed = std::signbit(targetAcc)
            ? std::max(targetSpeed, speed)
            : std::min(targetSpeed, speed);
        }
    }
    SpeedCmd Core::SetSpeedCmd(
        const bool force_speed, const float targetSpeed, const float acceleration)
    {
        SpeedCmd cmd;
        cmd.state_hint = true;
        cmd.force_speed = force_speed;
        cmd.speed = targetSpeed;
        cmd.acceleration = acceleration;

        return cmd;
    }

    void Core::DecisionState(
        const std::string tagState, itri_msgs::BehaviorState & behaviorState,
        const int trafficState, const float OAGenBiasObj, const float OASlowDownObj,
        const float IAObj, const float ACCObj, const bool OAGenBias)
    {
        switch(hash_run_time(tagState.c_str()))
        {
            case "LF"_hash:
                behaviorState.behavior_state = 2;
                behaviorState.obj_id = 0;
                behaviorState.obstacles = false;
                break;
            case "ACC"_hash:
                behaviorState.behavior_state = 9;
                behaviorState.obj_id = static_cast<int>(ACCObj);
                behaviorState.obstacles = true;
                break;
            case "OA"_hash:
                behaviorState.behavior_state = 4;
                behaviorState.obj_id = static_cast<int>(OASlowDownObj);
                behaviorState.obstacles = true;
                break;
            case "IA"_hash:
                behaviorState.behavior_state = 11;
                behaviorState.obj_id = static_cast<int>(IAObj);
                behaviorState.obstacles = true;
                break;
            case "Bumper"_hash:
                behaviorState.behavior_state = 10;
                behaviorState.obj_id = 99;
                behaviorState.obstacles = false;
                break;
            case "Curve"_hash:
                behaviorState.behavior_state = 15;
                behaviorState.obj_id = 98;
                behaviorState.obstacles = false;
                break;
            case "Traffic Light Left"_hash:
                behaviorState.behavior_state = 13;
                behaviorState.obj_id = trafficState;
                behaviorState.obstacles = false;
                break;
            case "Traffic Light Right"_hash:
                behaviorState.behavior_state = 14;
                behaviorState.obj_id = trafficState;
                behaviorState.obstacles = false;
                break;
            case "Traffic Light Red"_hash:
                behaviorState.behavior_state = 12;
                behaviorState.obj_id = trafficState;
                behaviorState.obstacles = false;
                break;
            case "PK"_hash:
                behaviorState.behavior_state = 6;
                behaviorState.obj_id = 0;
                behaviorState.obstacles = false;
                break;
            case "AEB"_hash:
                behaviorState.behavior_state = 1;
                behaviorState.obj_id = 0;
                behaviorState.obstacles = false;
                break;
        }

        behaviorState.avoidance = OAGenBias;
    }

    void Core::SetSpeedProfile(
        const std::map<std::string, SpeedCmd> & accelerationCmd,
        float & speed, float & targetAcc,float & targetSpeed,
        const double durationTime, std::string & tagState, State & behavior)
    {
        if (!accelerationCmd.empty())
        {
            if (accelerationCmd.count("AEB") > 0)
            {
                SpeedProtection(speed, targetAcc, targetSpeed,durationTime,
                    accelerationCmd.find("AEB")->second.speed,
                    accelerationCmd.find("AEB")->second.acceleration);
                tagState = "AEB";
                behavior = State::AEB;
                std::cout << "AAA AEB" << '\n';
            }
            else if (accelerationCmd.count("PK") > 0)
            {
                SpeedProtection(speed, targetAcc, targetSpeed,durationTime,
                    accelerationCmd.find("PK")->second.speed,
                    accelerationCmd.find("PK")->second.acceleration);
                tagState = "PK";
                behavior = State::PARKING;
                std::cout << "AAA PK" << '\n';
            }
            else
            {
                auto minAcceleration = MapGetMin(accelerationCmd);
                std::cout << minAcceleration.first <<
                    ", acceleration = " << minAcceleration.second.acceleration << std::endl;
                tagState = minAcceleration.first;

                SpeedProtection(speed, targetAcc, targetSpeed,durationTime,
                    minAcceleration.second.speed,
                    minAcceleration.second.acceleration);
            }
        }
        else
        {
            std::cout << "accelerationCmd is empty!!!!!" << '\n';
            speed += targetAcc;
            speed = std::signbit(targetAcc)
                ? std::max(targetSpeed, speed)
                : std::min(targetSpeed, speed);
        }
    }

    void Core::OrderClockwise(Polygon & poly)
    {
        TRACE_ASSERT_THROW(poly.corners.outer().size() > 2);

        PointXY point, center = {0.0, 0.0};
        std::vector<std::pair<PointXY, float>> polyOrder;
        std::pair<PointXY, float> corner;

        for (int i = 0; i < poly.corners.outer().size(); i++)
        {
            point = {boost::geometry::get<0>(poly.corners.outer()[i]),
                boost::geometry::get<1>(poly.corners.outer()[i])};
            center.x += point.x;
            center.y += point.y;
            corner = {point, 0.0};
            polyOrder.push_back(corner);
        }
        center.x /= poly.corners.outer().size();
        center.y /= poly.corners.outer().size();

        for (int i = 0; i < polyOrder.size(); i++)
        {
            polyOrder[i].second = GetPrincipalAngle(std::atan2(
                polyOrder[i].first.y - center.y, polyOrder[i].first.x - center.x));
        }

        std::sort(polyOrder.begin(), polyOrder.end(),
            [](const std::pair<PointXY, float> &lhs,
            const std::pair<PointXY, float> &rhs)
            {return lhs.second > rhs.second;});

        poly.corners.clear();
        for (int i = 0; i < polyOrder.size(); i++)
        {
            AddPoints(poly, {polyOrder[i].first.x, polyOrder[i].first.y});
        }
        boost::geometry::correct(poly.corners);
    }

    void Core::AddPoints(
        Polygon & pool, const BoostPoint & newpoint)
    {
        boost::geometry::append(pool.corners, newpoint);
    }

    Polygon Core::ObjListXYToPoly(const ObjectXY & object)
    {
        Polygon output;

        if (object.x.size() >= 4)
        {
            for (int i = 1; i < object.x.size(); i++)
            {
                boost::geometry::append(output.corners,
                    BoostPoint{object.x[i], object.y[i]});
            }
            OrderClockwise(output);
        }
        return output;
    }

    float Core::PolygonArea(const Polygon & poly)
    {
        return boost::geometry::area(poly.corners);
    }

    float Core::IntersectionArea(Polygon & poly1, Polygon & poly2)
    {
        std::deque<BoostPolygon> output;
        //OrderClockwise(poly1);
        //OrderClockwise(poly2);
        boost::geometry::intersection(poly1.corners, poly2.corners, output);

        float area = 0.0;;
        BOOST_FOREACH(BoostPolygon const& p, output)
        {
            area += boost::geometry::area(p);
        }
        return area;
    }

    bool Core::isPointInPolygon(PointXY & point, Polygon & polygon)
    {
        return (boost::geometry::within(BoostPoint{point.x, point.y}, polygon.corners) ? true : false);
    }

    Polygon Core::LaneToPoly(const itri_msgs::WaypointArray & lane, const float roadWidth)
    {
        Polygon output;
        float pointx, pointy, heading;

        if (lane.waypoints.size() > 1)
        {
            std::vector<PointXY> leftPoints;
    		std::vector<PointXY> rightPoints;
            std::vector<PointXY> lanePoints;
            std::vector<float> curvature;

            CalculatePathCurvature(lane, curvature);
    		PathOffset(lane, -roadWidth, curvature, rightPoints);
    		PathOffset(lane, roadWidth, curvature, leftPoints);

            std::reverse(rightPoints.begin(),rightPoints.end());
            lanePoints.insert(lanePoints.end(), leftPoints.begin(),leftPoints.end());
            lanePoints.insert(lanePoints.end(), rightPoints.begin(),rightPoints.end());

    		// std::reverse(leftPoints.begin(),leftPoints.end());
            // lanePoints.insert(lanePoints.end(), rightPoints.begin(),rightPoints.end());
            // lanePoints.insert(lanePoints.end(), leftPoints.begin(),leftPoints.end());

    		for (int i = 0; i < lanePoints.size(); i++)
            {
                boost::geometry::append(output.corners,
                    BoostPoint{lanePoints[i].x, lanePoints[i].y});
            }

            boost::geometry::correct(output.corners);
        }
        return output;
    }

    void Core::CalculatePathCurvature(const itri_msgs::WaypointArray & path, std::vector<float> & curvature)
    {
    	curvature.resize(path.waypoints.size(), 0.0f);

    	for (size_t i = 1; i < curvature.size(); i ++)
        {
            const float angleHalf = HALF * SupplementaryAngle(
                path.waypoints[i].pose.pose.orientation.z -
                path.waypoints[i - 1].pose.pose.orientation.z);
            const float radius = HALF * RESOLUTION * std::tan(angleHalf);
            curvature[i] = std::pow(radius, -1.0f);
        }
    }

    void Core::PathOffset(
        const itri_msgs::WaypointArray & Oldpath,
        const float bias,
        const std::vector<float> curvature,
        std::vector<PointXY> & newPath)
    {
        newPath.clear();

        for (size_t j = 0; j < Oldpath.waypoints.size(); ++ j)
        {
            const float offset = bias;

            if (IsCurveUnderThreshold(curvature[j], offset))
            {
                itri_msgs::Waypoint point = Oldpath.waypoints[j];
                PointOffset(offset, point);
    			PointXY pointOutput =
                    {point.pose.pose.position.x, point.pose.pose.position.y};
                newPath.push_back(pointOutput);
            }
        }
    }

    bool Core::IsCurveUnderThreshold(
        const float curvature, const float offset)
    {
        const float thershold = 1.0f / (std::abs(offset) + THRESHOLD_SAFETY_FACTOR);
        return std::signbit(curvature * offset) || std::abs(curvature) < thershold;
    }

    void Core::PointOffset(
        const float distance, itri_msgs::Waypoint & point)
    {
        float dir = copysign(-1.0, distance);
        point.pose.pose.position.x +=
            std::abs(distance) * std::cos(point.pose.pose.orientation.z + dir * M_PI_2);
        point.pose.pose.position.y +=
            std::abs(distance) * std::sin(point.pose.pose.orientation.z + dir * M_PI_2);
    }
}/* namespace Behavior */
