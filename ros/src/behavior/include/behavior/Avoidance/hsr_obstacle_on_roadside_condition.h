#ifndef OBJ_ON_ROADSIDE_CONDITION_H
#define OBJ_ON_ROADSIDE_CONDITION_H

#include <condition_node.h>
#include <behavior/obstacle_avoidance.h>
#include <behavior/Avoidance/hsr_avoidance_sequence.h>

namespace Behavior
{
    class objOnRoadsideCondition : public BT::ConditionNode
    {
    public:
        objOnRoadsideCondition(
            std::string name,
            const std::shared_ptr<BehaviorOutput> & nodeCommand,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context,
            hsrAvoidanceSequence & nodeContext);
        BT::ReturnStatus Tick();

        std::vector<ObjectSD> FrontObstaclesInLocalPath();
        void SeperateObstacles(std::vector<ObjectSD> & frontObstacles);
        float FindObjSideRatio(const ObjectSD frontObject);
        bool ObstacleIsStatic(const ObjectSD frontObject);

    private:
        const std::shared_ptr<BehaviorOutput> mNodeCommand;
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
        hsrAvoidanceSequence & mNodeContext;

        float mRollInDistance;
    };
}

#endif
