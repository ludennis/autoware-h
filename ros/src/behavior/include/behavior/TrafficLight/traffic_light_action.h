#ifndef TRAFFIC_LIGHT_ACTION_H
#define TRAFFIC_LIGHT_ACTION_H

#include <action_node.h>
#include <behavior/types.h>
#include <behavior/context.h>
#include <string>

namespace Behavior
{
    class TrafficLightAction : public BT::ActionNode
    {
    public:
        TrafficLightAction(
            std::string name,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context);
        BT::ReturnStatus Tick();
        void Halt();
    private:
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
        const std::string mName;
    protected:
        itri_msgs::plan mBehaviorPlanOut;
    };
}

#endif
