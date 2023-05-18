#ifndef __LC_LANE_CHANGE_SEQUENCE_H__
#define __LC_LANE_CHANGE_SEQUENCE_H__

#include <behavior/context.h>
#include <behavior/lane_change/lane_change_context.h>
#include <memory>
#include <sequence_node.h>

namespace Behavior
{
    class LaneChangeSequence : public BT::SequenceNode
    {
    public:
        LaneChangeSequence(const std::string & name,
            const std::shared_ptr<Context> & context);

    private:
        const std::shared_ptr<Context> mContext;
        const std::shared_ptr<LaneChangeContext> mLaneChangeContext;
    };
} // namespace Behavior

#endif // __LC_LANE_CHANGE_SEQUENCE_H__
