#include <behavior/Cruise/bumper_condition.h>
#include <string>
#include <mutex>

#define _BUMPER_DEBUG_
namespace Behavior
{
    BumperCondition::BumperCondition(
        std::string name,
        const std::shared_ptr<Context> & context)
        : ConditionNode::ConditionNode(name)
        , mContext(context)
    {

    }

    BT::ReturnStatus BumperCondition::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);
        bool bumpCondition =
            ((static_cast<float>(mContext->currentBumper - mContext->vehPosSD.s) <
            std::max(25.0f, std::pow(mContext->maxSpeed, 2.0f) / 2.0f)) &&
            (static_cast<float>(mContext->currentBumper - mContext->vehPosSD.s)) > -3.0f);
#ifdef _BUMPER_DEBUG_
        if (bumpCondition)
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " has succeeded");
        }
        else
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " is failed");
        }
#endif
        return bumpCondition ? BT::SUCCESS : BT::FAILURE;
    }
}
