#include <behavior/ACC/acc_condition.h>
#include <string>
#include <mutex>

namespace Behavior
{
    ACCCondition::ACCCondition(
        std::string name,
        const std::shared_ptr<Context> & context)
        : ConditionNode::ConditionNode(name)
        , mContext(context)
    {

    }

    BT::ReturnStatus ACCCondition::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);
#ifdef _ACC_DEBUG
        if (mContext->obstacleHazard)
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " has succeeded");
        }
        else
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " is failed");
        }
#endif
        return mContext->obstacleHazard ? BT::SUCCESS : BT::FAILURE;
    }

}
