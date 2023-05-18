#ifndef TL_LEFT_CONDITION_H
#define TL_LEFT_CONDITION_H

#include <condition_node.h>
#include <behavior/types.h>
#include <behavior/context.h>

namespace Behavior
{
    class TLLeftCondition : public BT::ConditionNode
    {
    public:
        TLLeftCondition(
            std::string name,
            const std::shared_ptr<Context> & context);
        BT::ReturnStatus Tick();

    private:
        const std::shared_ptr<Context> mContext;
    };
}

#endif
