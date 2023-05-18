#ifndef AVOIDANCE_FALLBACK_H
#define AVOIDANCE_FALLBACK_H

#include <fallback_node.h>
#include <behavior/context.h>

namespace Behavior
{
    class AvoidanceFallback : public BT::FallbackNode
    {
    public:
        AvoidanceFallback(std::string name,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context);

    private:
        const std::shared_ptr<BehaviorOutput> mNodeCommand;
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
    };
}

#endif
