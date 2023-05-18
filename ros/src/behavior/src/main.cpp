#include <behavior/state_machine_node.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior");

    Behavior::StateMachineNode node;

    node.MainLoop();

    return 0;
}
