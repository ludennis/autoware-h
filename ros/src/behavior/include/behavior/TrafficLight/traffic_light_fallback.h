#ifndef BEHAVIORTREE_TRAFFIC_LIGHT_SEQUENCE_H
#define BEHAVIORTREE_TRAFFIC_LIGHT_SEQUENCE_H

#include <fallback_node.h>
#include <behavior/context.h>
#include <behavior/types.h>
#include <string>

#define DEFINE_PROPERTY(type, name) \
    private: \
        type m ## name; \
    public: \
        inline type Get ## name() { return m ## name; } \
        inline void Set ## name(const type value) { m ## name = value; } \

namespace Behavior
{
    class TrafficLightFallback : public BT::FallbackNode
    {
    public:
        TrafficLightFallback(
            std::string name,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context);
    private:
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;

    };
}

#endif
