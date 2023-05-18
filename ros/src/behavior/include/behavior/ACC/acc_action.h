#ifndef ACC_ACTION_H
#define ACC_ACTION_H

#include <action_node.h>
#include <behavior/types.h>
#include <behavior/context.h>

namespace Behavior
{
    class ACCAction : public BT::ActionNode
    {
    public:
        ACCAction(
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
