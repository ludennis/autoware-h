#ifndef BEHAVIORTREE_AVOIDANCE_H
#define BEHAVIORTREE_AVOIDANCE_H

#include <sequence_node_with_memory.h>
#include <behavior/context.h>

#define DEFINE_PROPERTY(type, name) \
    private: \
        type m ## name; \
    public: \
        inline type Get ## name() { return m ## name; } \
        inline void Set ## name(const type value) { m ## name = value; } \

namespace Behavior
{
    class AvoidanceSequence : public BT::SequenceNodeWithMemory
    {
    public:
        static const float OBJECTS_SEARCHING_RANGE;
        static const float MIN_CAR_SPEED;
        static const float DEFAULT_ACCELERATION;
        static const float STOP_SPEED;

    public:
        AvoidanceSequence(std::string name,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context);

        DEFINE_PROPERTY(float, TargetPoint);
        DEFINE_PROPERTY(float, TargetD);
        DEFINE_PROPERTY(float, BiasResolution);
        DEFINE_PROPERTY(float, WaitingTime);
        DEFINE_PROPERTY(double, ObstacleStayTime);
        DEFINE_PROPERTY(ObjectSD, FrontObstacle);
        DEFINE_PROPERTY(double, InterruptTime);
        DEFINE_PROPERTY(SpeedCmd, SpeedCommand);

        void EnsureMinDrivingSpeed();

    private:
        const std::shared_ptr<BehaviorOutput> mNodeCommand;
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
    };
}

#endif
