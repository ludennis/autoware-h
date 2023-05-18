#ifndef __BEHAVIOR_LANE_FOLLOWING_H__
#define __BEHAVIOR_LANE_FOLLOWING_H__

#include <behavior/core.h>
#include <behavior/lane_type.h>
#include <visualization_msgs/MarkerArray.h>

namespace Behavior
{

    class LaneFollowing : public Core
    {
        enum class State
        {
            INITIAL = 0,
            LANE_CHANGE,
            OBSTACLE_AVOID,
            OBSTACLE_HAZARD,
            CURVE,
            SPEEDUP,
            STABLE,
            BUMP,
            CROSS_ROAD,
            TRAFFIC_LIGHT_RED,
            TRAFFIC_LIGHT_YELLOW,
            TRAFFIC_LIGHT_WAIT_FOR_TURN_LEFT,
            TRAFFIC_LIGHT_WAIT_FOR_TURN_RIGHT,
            HUMAN_MODE, //12
            DONOTHING,
        };

    public:
        LaneFollowing();
        void PIDRelease();
        void PIDInit(float kp, float ki, float kd);
        void UpdatePIDError(
            const float vehSpeed,
            const float shortestDistsnce,
            float & relaVelocity,
            float & deltaDistance);
        void PIDController(float & controlVelocity);
        float ACCState(
            const float prevCmdSpeed,
            const float carSpeed,
            const float maxSpeed,
            float & shortestDistsnce,
            float & relaVelocity,
            const float pidPGain,
            const float pidIGain,
            const float pidDGain,
            float & deltaDistance);
        void CurveGetStartEnd(
            const std::vector<float> & globalPathCurvature,
            std::vector<int> & indexStartCurve,
            std::vector<int> & indexEndCurve);
        float ForceSpeedByDistance(
            float trafficObjIndex,
            const float targetSpeed,
            const float vehPosS,
            const float carSpeed,
            const float lastSpeedCmd,
            const float maxSpeed,
            const double timeStep);
        void UpdateTrafficStopLine(
            int & currentStopLine,
            const float vehPosS,
            const int nextTrafficStopLine,
            const int trafficLightStatus);
        bool CurveAvoid(
            const float vehiclePositionS,
            const std::vector<float> & globalPathS,
            const std::vector<float> & globalPathCurvature,
            const std::vector<int> & indexStartCurve,
            const std::vector<int> & indexEndCurve,
            const int curveNum,
            const float maxSpeed,
            float & curveSpeed);
        void CurveSpeed(
            const float carSpeed,
            SpeedCmd & LFOutput,
            const float curveSpeed);
        float TerminlDistance(const float carSpeed, const float terminalSpeed);
        int DoPathMatching(
            const std::vector<float> & pathX,
            const std::vector<float> & pathY,
            const float x,
            const float y);
        int ClosestObjOnLocalPath(
            const float maxSpeed,
            const std::vector<ObjectXY> & objListXY,
            const std::vector<ObjectSD> & objListSD,
            Polygon & accRoiPoly,
            const VehPosSD vehPos,
            bool & obstacleHazard,
            float & shortestDistsnce,
            float & frontVelocity,
            visualization_msgs::MarkerArray & LFMarkerArray,
            std::vector<ObjectXY> & ACCFrontObjs,
            ObjectXY & ACCObj);
        int NextCurveIndex(
            const float vehicleS,
            const std::vector<int> & indexEndCurve,
            const std::vector<float> & globalPathS);
        void ACCOutput(
            const float prevCmdSpeed,
            const float carSpeed,
            const float maxSpeed,
            const float curveSpeed,
            const bool bumpHint,
            const bool enterCurve,
            float & shortestDistsnce,
            float & relaVelocity,
            const double & timeStep,
            const float pidPGain,
            const float pidIGain,
            const float pidDGain,
            float & accSpeed,
            float & controlAccelerate,
            float & deltaDistance);
        void VelocitySetPoint(
            const float carSpeed,
            const float curveSpeed,
            const bool bumpHint,
            const bool enterCurve,
            const float maxSpeed,
            float & frontVelocity);
        visualization_msgs::Marker PubRvizLFObj(
            const int objID, const std::vector<ObjectXY> & objListXY,
            const float r, const float g, const float b);
        double mPGain;
        double mIGain;
        double mDGain;
    private:

        ros::NodeHandle mLFNode;
        State mState;

        std::vector<int> mIndexStartCurve;
        std::vector<int> mIndexEndCurve;
        int mCurveNum;
        bool mCalCurvePoint;
        int mCurrentStopLine;
        int mCurrentBumper;
        int mAccObjID;
        ros::Publisher mPubRvizLFObj;

        bool mMaintainingDistance;
        bool mAccTrigger;
        float mPrevSpeed;
        float mPrevSetPoint;
        float mKp;
        float mKd;
        float mKi;
        float mPError;
        float mIError;
        float mDError;
    };//class
}

#endif // __BEHAVIOR_LANE_FOLLOWING_H__
