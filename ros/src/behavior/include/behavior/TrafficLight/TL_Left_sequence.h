#ifndef TRAFFIC_LIGHT_LEFT_SEQUENCE_H
#define TRAFFIC_LIGHT_LEFT_SEQUENCE_H

#include <sequence_node.h>
#include <behavior/context.h>
#include <behavior/types.h>

namespace Behavior
{
    class TLLeftSequence : public BT::SequenceNode
    {
    public:
        TLLeftSequence(
            std::string name,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context);
    private:
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
    };
}

#endif
