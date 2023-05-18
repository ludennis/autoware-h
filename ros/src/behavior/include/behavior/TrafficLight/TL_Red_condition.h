#ifndef TL_RED_CONDITION_H
#define TL_RED_CONDITION_H

#include <condition_node.h>
#include <behavior/types.h>
#include <behavior/context.h>

namespace Behavior
{
    class TLRedCondition : public BT::ConditionNode
    {
    public:
        TLRedCondition(
            std::string name,
            const std::shared_ptr<Context> & context);
        BT::ReturnStatus Tick();

    private:
        const std::shared_ptr<Context> mContext;
    };
}

#endif
