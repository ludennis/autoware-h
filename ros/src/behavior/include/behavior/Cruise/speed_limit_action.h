#ifndef SPEED_LIMIT_ACTION_H
#define SPEED_LIMIT_ACTION_H

#include <action_node.h>
#include <behavior/types.h>
#include <behavior/context.h>

namespace Behavior
{
    class SpeedLimitAction : public BT::ActionNode
    {
    public:
        SpeedLimitAction(
            std::string name,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context);
        BT::ReturnStatus Tick();
        void Halt();
    private:
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
    protected:
        itri_msgs::plan mBehaviorPlanOut;
    };
}

#endif
