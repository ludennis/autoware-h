#ifndef AEB_TRIGGERED_CONDITION_H
#define AEB_TRIGGERED_CONDITION_H

#include <action_node.h>
#include <behavior/context.h>
#include <behavior/aeb.h>

namespace Behavior
{
    class AebTriggeredAction: public BT::ActionNode
    {
    public:
        AebTriggeredAction(
            std::string name,
            const std::shared_ptr<Context> & context);
        BT::ReturnStatus Tick();
        void Halt();

    private:
        const std::shared_ptr<Context> mContext;
    };
}

#endif
