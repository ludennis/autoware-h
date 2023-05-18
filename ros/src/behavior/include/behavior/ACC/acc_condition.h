#ifndef ACC_CONDITION_H
#define ACC_CONDITION_H

#include <condition_node.h>
#include <behavior/types.h>
#include <behavior/context.h>

namespace Behavior
{
    class ACCCondition : public BT::ConditionNode
    {
    public:
        ACCCondition(
            std::string name,
            const std::shared_ptr<Context> & context);
        BT::ReturnStatus Tick();

    private:
        const std::shared_ptr<Context> mContext;
    };
}

#endif
