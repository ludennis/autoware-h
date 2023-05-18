#ifndef __UTLIS_H__
#define __UTLIS_H__

#include <geometry_msgs/Pose.h>
#include <itri_msgs/WaypointArray.h>

namespace MotionPlannerUtils
{
    struct PlannerParam
    {
        double weightCenter;
        double weightTransition;
        double weightCollision;
        double wayptRes;
        double rollOutDist;
        double rollInFactor;

        PlannerParam()
            : weightCenter(0.0)
            , weightTransition(0.0)
            , weightCollision(0.0)
            , wayptRes(0.0)
            , rollOutDist(0.0)
            , rollInFactor(0.0)
        {}
    };

    int PathMatching(
        const itri_msgs::WaypointArray & globalPath,
        const geometry_msgs::Pose & point);

    void SetPathHeadingAngle(itri_msgs::WaypointArray & path);

    void PathRegularize(
        const float resolution, const itri_msgs::WaypointArray & oldPath,
        itri_msgs::WaypointArray & newPath);

    std::vector<int> Histogram(
        const std::vector<float> & array, const float bucketSize);

    float LateralProjection(
        const float x, const float y, const float angle);

    float LongitudinalProjection(
        const float x, const float y, const float angle);

} // namespace MotionPlannerUtils

#endif // __UTLIS_H__
