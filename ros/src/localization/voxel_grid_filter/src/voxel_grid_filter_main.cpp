#include <voxel_grid_filter.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, ros::this_node::getName());

  VoxelGridFilter voxelGridFilter;
  voxelGridFilter.Run();

  return 0;
}
