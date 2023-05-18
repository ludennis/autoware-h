#ifndef HSR_AVOIDANCE_SEQUENCE_H
#define HSR_AVOIDANCE_SEQUENCE_H

#include <sequence_node.h>
#include <behavior/context.h>

#define DEFINE_PROPERTY(type, name) \
    private: \
        type m ## name; \
    public: \
        inline type Get ## name() { return m ## name; } \
        inline void Set ## name(const type value) { m ## name = value; } \

namespace Behavior
{
    class hsrAvoidanceSequence : public BT::SequenceNode
    {
    public:
        hsrAvoidanceSequence(std::string name,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context);

        DEFINE_PROPERTY(bool, CheckRightArea);
        DEFINE_PROPERTY(bool, CheckLeftArea);
        DEFINE_PROPERTY(std::vector<ObjectSD>, SideObstacles);
        DEFINE_PROPERTY(std::vector<ObjectSD>, OnRoadObstacles);
        DEFINE_PROPERTY(float, Bias);

    private:
        const std::shared_ptr<BehaviorOutput> mNodeCommand;
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
    };
}

#endif
