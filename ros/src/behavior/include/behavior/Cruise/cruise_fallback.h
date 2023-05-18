#ifndef BEHAVIOR_CRUISE_FALLBACK_H
#define BEHAVIOR_CRUISE_FALLBACK_H

#include <fallback_node.h>
#include <sequence_node.h>
#include <parallel_node.h>
#include <behavior/context.h>

namespace Behavior
{
    class CruiseFallback : public BT::FallbackNode
    {
    public:
        CruiseFallback(
            std::string name,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context);

    private:
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
    };
}

#endif
