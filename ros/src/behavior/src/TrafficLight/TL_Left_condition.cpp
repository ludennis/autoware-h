#include <behavior/TrafficLight/TL_Left_condition.h>
#include <string>
#include <mutex>

#define _TL_LEFT_DEBUG_
namespace Behavior
{
    TLLeftCondition::TLLeftCondition(
        std::string name,
        const std::shared_ptr<Context> & context)
        : ConditionNode::ConditionNode(name)
        , mContext(context)
    {

    }

    BT::ReturnStatus TLLeftCondition::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);
        bool tfLeftCondition =
            ((mContext->currentStopLine == 714 || mContext->currentStopLine == 8581 ||
            mContext->currentStopLine == 15351) &&
            (static_cast<float>(std::abs(mContext->currentStopLine - mContext->vehPosSD.s)) <
            std::max(30.0f, std::pow(mContext->vehPosSD.speed, 2.0f) / (2.0f * 0.5f))) &&
            (mContext->trafficLight != 5 && mContext->trafficLight != 8 &&
            mContext->trafficLight != 11 && mContext->trafficLight != 13));
#ifdef _TL_LEFT_DEBUG_
        if (tfLeftCondition)
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " has succeeded");
        }
        else
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " is failed");
        }
#endif
        return tfLeftCondition ? BT::SUCCESS : BT::FAILURE;
    }
}
