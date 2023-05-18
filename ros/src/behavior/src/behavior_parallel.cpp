#include <behavior/behavior_parallel.h>
#include <string>
#include <control_node.h>
#include <fallback_node.h>
#include <parallel_node.h>
#include <behavior/Avoidance/hsr_avoidance_sequence.h>
#include <behavior/ACC/acc_sequence.h>
#include <behavior/Cruise/cruise_fallback.h>
#include <behavior/TrafficLight/traffic_light_fallback.h>

namespace Behavior
{
    BehaviorParallel::BehaviorParallel(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : ParallelNode::ParallelNode(name,10)
        , mNodeCommand(std::make_shared<BehaviorOutput>())
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {
        AddChild(std::make_shared<hsrAvoidanceSequence>(
            "Avoidance Sequence", mNodeSpeedCommand, mContext));
        AddChild(std::make_shared<TrafficLightFallback>(
            "TrafficLight Fallback", mNodeSpeedCommand, mContext));
        AddChild(std::make_shared<ACCSequence>(
            "ACC Sequence", mNodeSpeedCommand, mContext));
        AddChild(std::make_shared<CruiseFallback>(
            "Cruise Fallback", mNodeSpeedCommand, mContext));
    }
}
