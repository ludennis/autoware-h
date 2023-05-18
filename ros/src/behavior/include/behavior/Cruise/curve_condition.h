#ifndef CURVE_CONDITION_H
#define CURVE_CONDITION_H

#include <condition_node.h>
#include <behavior/types.h>
#include <behavior/context.h>

namespace Behavior
{
    class CurveCondition : public BT::ConditionNode
    {
    public:
        CurveCondition(
            std::string name,
            const std::shared_ptr<Context> & context);
        BT::ReturnStatus Tick();

    private:
        const std::shared_ptr<Context> mContext;
    };
}

#endif
