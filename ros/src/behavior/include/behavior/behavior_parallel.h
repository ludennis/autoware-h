#ifndef BEHAVIOR_PARALLEL_H
#define BEHAVIOR_PARALLEL_H

#include <fallback_node.h>
#include <sequence_node.h>
#include <parallel_node.h>
#include <behavior/context.h>

namespace Behavior
{
    class BehaviorParallel : public BT::ParallelNode
    {
    public:
        BehaviorParallel(std::string name,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context);

    private:
        const std::shared_ptr<BehaviorOutput> mNodeCommand;
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
    };
}

#endif
