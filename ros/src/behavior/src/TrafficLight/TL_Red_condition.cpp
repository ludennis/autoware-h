#include <behavior/TrafficLight/TL_Red_condition.h>
#include <string>
#include <mutex>

#define _TL_RED_DEBUG_
namespace Behavior
{
    TLRedCondition::TLRedCondition(
        std::string name,
        const std::shared_ptr<Context> & context)
        : ConditionNode::ConditionNode(name)
        , mContext(context)
    {

    }

    BT::ReturnStatus TLRedCondition::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);
        bool tfRedCondition =
            ((mContext->trafficLight == 1 || (mContext->currentStopLine == 7277 &&
            mContext->trafficLight != 2 && mContext->trafficLight != 4 &&
            mContext->trafficLight != 8) || (mContext->trafficLight == 3 &&
            (static_cast<float>(std::abs(mContext->currentStopLine - mContext->vehPosSD.s)) >
            mContext->vehPosSD.speed * 1.0f))) &&
            (static_cast<float>(std::abs(mContext->currentStopLine - mContext->vehPosSD.s)) <
            std::max(30.0f, std::pow(mContext->vehPosSD.speed, 2.0f) / (2.0f * 0.5f))));
#ifdef _TL_LEFT_DEBUG_
        if (tfRedCondition)
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " has succeeded");
        }
        else
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " is failed");
        }
#endif
        return tfRedCondition ? BT::SUCCESS : BT::FAILURE;
    }
}
