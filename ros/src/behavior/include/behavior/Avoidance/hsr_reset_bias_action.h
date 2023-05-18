#ifndef RESET_BIAS_ACTION_H
#define RESET_BIAS_ACTION_H

#include <action_node.h>
#include <behavior/obstacle_avoidance.h>
#include <behavior/Avoidance/hsr_avoidance_sequence.h>

namespace Behavior
{
    class resetBiasAction : public BT::ActionNode
    {
    public:
        resetBiasAction(
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

    protected:
        itri_msgs::plan mBehaviorPlanOut;
        ros::NodeHandle mNode;
    };
}

#endif
