/*
    A smoother that smoothes out the noise of ndt_pose
*/

#include <pose_smoother_node.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "pose_smoother_node");

    PoseSmootherNode smoother;

    smoother.Run();

    return 0;
}
