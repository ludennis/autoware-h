#include <ros/init.h>
#include <motion_planner/motion_planner_node.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planner");
  MotionPlannerNode motionPlannerNode;

  motionPlannerNode.MainLoop();

  return(0);
}
