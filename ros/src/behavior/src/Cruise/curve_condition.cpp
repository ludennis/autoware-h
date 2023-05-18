#include <behavior/Cruise/curve_condition.h>
#include <string>
#include <mutex>

namespace Behavior
{
    CurveCondition::CurveCondition(
        std::string name,
        const std::shared_ptr<Context> & context)
        : ConditionNode::ConditionNode(name)
        , mContext(context)
    {

    }

    BT::ReturnStatus CurveCondition::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);
        if (mContext->enterCurve)
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " has succeeded");
        }
        else
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " is failed");
        }
        return mContext->enterCurve ? BT::SUCCESS : BT::FAILURE;
    }
}
