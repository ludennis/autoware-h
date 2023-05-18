#ifndef GEN_BIAS_ACTION_H
#define GEN_BIAS_ACTION_H

#include <action_node.h>
#include <behavior/obstacle_avoidance.h>
#include <behavior/Avoidance/hsr_avoidance_sequence.h>
#include <visualization_msgs/MarkerArray.h>

namespace Behavior
{
    class genBiasAction : public BT::ActionNode
    {
    public:
        genBiasAction(
            std::string name,
            const std::shared_ptr<BehaviorOutput> & nodeCommand,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context,
            hsrAvoidanceSequence & nodeContext);
        BT::ReturnStatus Tick();
        void Halt();

        float FindMaxBias();
        bool WithoutBackObstacle(const float bias);
        bool IsBiasNeedToChange(const float bias);
        bool IsNewPathSafe(const float bias);

    private:
        const std::shared_ptr<BehaviorOutput> mNodeCommand;
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
        hsrAvoidanceSequence & mNodeContext;

    protected:
        itri_msgs::plan mBehaviorPlanOut;
        ros::NodeHandle mNode;
        void Visualization();
    };
}

#endif
