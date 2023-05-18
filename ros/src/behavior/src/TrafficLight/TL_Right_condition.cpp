#include <behavior/TrafficLight/TL_Right_condition.h>
#include <string>
#include <mutex>

#define _TL_RIGHT_DEBUG_
namespace Behavior
{
    TLRightCondition::TLRightCondition(
        std::string name,
        const std::shared_ptr<Context> & context)
        : ConditionNode::ConditionNode(name)
        , mContext(context)
    {

    }

    BT::ReturnStatus TLRightCondition::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);
        bool tfRightCondition =
            ((mContext->currentStopLine == 278 || mContext->currentStopLine == 4364 ||
            mContext->currentStopLine == 14895) &&
            (static_cast<float>(std::abs(mContext->currentStopLine - mContext->vehPosSD.s)) <
            std::max(30.0f, std::pow(mContext->vehPosSD.speed, 2.0f) / (2.0f * 0.5f))) &&
            (mContext->trafficLight != 2 && mContext->trafficLight != 6 &&
            mContext->trafficLight != 9 && mContext->trafficLight != 12));
#ifdef _TL_LEFT_DEBUG_
        if (tfRightCondition)
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " has succeeded");
        }
        else
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " is failed");
        }
#endif
        return tfRightCondition ? BT::SUCCESS : BT::FAILURE;
    }
}
