#ifndef __BEHAVIOR_DEFINE_H__
#define __BEHAVIOR_DEFINE_H__

#include <behavior/lane_type.h>
#include <stdint.h>
#include <vector>

namespace Behavior
{
    struct BehaviorOutput
    {
        bool hint;
        uint8_t numLevel;
        float dev;
        float bias;
        float terminal_speed;
        float terminal_distance;
        bool update_num_level;
        bool update_dev;
        bool update_bias;
        bool update_terminal_speed;
        bool update_terminal_distance;
        bool parking_start;

        BehaviorOutput()
            : hint(false)
            , numLevel(0)
            , dev(0.0f)
            , bias(0.0f)
            , terminal_speed(0.0f)
            , terminal_distance(0.0f)
            , update_num_level(false)
            , update_dev(false)
            , update_bias(false)
            , update_terminal_speed(false)
            , update_terminal_distance(false)
            , parking_start(false)
        {}
    };

    struct SpeedCmd
    {
        bool state_hint;
        bool force_speed;
        float speed;
        float acceleration;

        SpeedCmd()
            : state_hint(false)
            , force_speed(false)
            , speed(0.0f)
            , acceleration(0.0f)
        {}
    };

    struct ObjectXY
    {
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> z;
        uint32_t state;
        float centerX;
        float centerY;
        float id;
        float speedX;
        float speedY;
        float relativeSpeedX;
        float relativeSpeedY;
    };

    struct ROIXY
    {
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> z;
        int id;
    };


    struct ObjectSD
    {
        std::vector<float> s;
        std::vector<float> d;
        uint32_t state;
        float centerS;
        float centerD;
        float id;
        float speedS;
        float speedD;
        float relativeSpeedS;
        float relativeSpeedD;
    };

    struct PointXY
    {
        float x;
        float y;
    };

    struct PointsSD
    {
        std::vector<float> s;
        std::vector<float> d;
    };

    struct ObjectHints
    {
        float frontObjDistance;
        float backObjDistance;
        bool leftSideIsSafe;
        bool rightSideIsSafe;
    };

    struct VehPosSD
    {
        float s;
        float d;
        float heading;
        float speed;
        float yawRate;
    };

    struct VehPosXY
    {
        float x;
        float y;
        float heading;
        float speed;
        float yawRate;
        bool stable;
    };

    struct ObjRectBound
    {
        float minS;
        float maxS;
        float centerS;
        float centerD;
        float leftBound;
        float rightBound;
    };

    struct ObjOnPath
    {
        bool onPath;
        int index;
    };

    struct VehicleOdometry
    {
        float speed;
        float steering;
        bool auto_mode;
    };

    struct LaneInfo
    {
        float laneWidth;
        int laneNum;
        int laneID;
        LaneType leftType;
        LaneType rightType;
    };
}/* namespace Behavior */

#endif /* __BEHAVIOR_DEFINE_H__ */
