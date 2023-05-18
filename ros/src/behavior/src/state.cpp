#include <behavior/state.h>
#include <trace/utils.h>

#define TRACE_TAG "Behavior.State"

namespace Behavior {

    const char * ToString(const State state)
    {
        switch (state)
        {
        #define BS(name) case State::name: return #name;
            BEHAVIOR_STATES
        #undef BS
        default:
            TRACE_FAIL_THROW();
            return nullptr;
        }
    }

} // namespace Behavior
