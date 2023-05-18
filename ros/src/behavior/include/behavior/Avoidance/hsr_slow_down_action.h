#ifndef HSR_SLOW_DOWN_ACTION_H
#define HSR_SLOW_DOWN_ACTION_H

#include <action_node.h>
#include <behavior/obstacle_avoidance.h>
#include <behavior/Avoidance/hsr_avoidance_sequence.h>
#include <map>

namespace Behavior
{
    class hsrSlowDownAction : public BT::ActionNode
    {
    public:
        hsrSlowDownAction(
            std::string name,
            const std::shared_ptr<BehaviorOutput> & nodeCommand,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context,
            hsrAvoidanceSequence & nodeContext);
        BT::ReturnStatus Tick();
        void Halt();

    private:
        const std::shared_ptr<BehaviorOutput> mNodeCommand;
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
        hsrAvoidanceSequence & mNodeContext;
        std::map<int, float> mAccelerationCmdList;

    protected:
        void Visualization();
        itri_msgs::plan mBehaviorPlanOut;
    };
}

#endif
