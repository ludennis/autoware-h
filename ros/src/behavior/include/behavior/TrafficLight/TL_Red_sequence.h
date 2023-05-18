#ifndef TRAFFIC_LIGHT_RED_SEQUENCE_H
#define TRAFFIC_LIGHT_RED_SEQUENCE_H

#include <sequence_node.h>
#include <behavior/context.h>
#include <behavior/types.h>

namespace Behavior
{
    class TLRedSequence : public BT::SequenceNode
    {
    public:
        TLRedSequence(
            std::string name,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context);
    private:
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
    };
}

#endif
