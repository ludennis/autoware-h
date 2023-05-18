#ifndef BEHAVIORTREE_ACC_SEQUENCE_H
#define BEHAVIORTREE_ACC_SEQUENCE_H

#include <sequence_node.h>
#include <behavior/context.h>
#include <behavior/types.h>

#define DEFINE_PROPERTY(type, name) \
    private: \
        type m ## name; \
    public: \
        inline type Get ## name() { return m ## name; } \
        inline void Set ## name(const type value) { m ## name = value; } \

namespace Behavior
{
    class ACCSequence : public BT::SequenceNode
    {
    public:
        ACCSequence(
            std::string name,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context);
    private:
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;

    };
}

#endif
