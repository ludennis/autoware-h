/*
 * Copyright (c) 2014, Nagoya University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Autoware nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VOXEL_GRID_FILTER_H
#define VOXEL_GRID_FILTER_H

#include <chrono>
#include <itri_msgs/ConfigVoxelGridFilter.h>
#include <itri_msgs/VoxelGridFilterInfo.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/*
 * A filter that downsamples a point cloud while maintining strong geometric
 * features by creating voxel grids specified by the voxel leaf size.
 *
 * Refactored class according to file:
 * https://github.com/autowarefoundation/autoware/blob/release/
 *  1.8.1/ros/src/sensing/filters/packages/
 *  points_downsampler/nodes/voxel_grid_filter/voxel_grid_filter.cpp
 */
class VoxelGridFilter
{
public:
  VoxelGridFilter();
  void Run();
private:
  pcl::PointCloud<pcl::PointXYZI> RemovePointsByRange(
    pcl::PointCloud<pcl::PointXYZI> scan, double minRange, double maxRange);
  ros::NodeHandle mNh;
  ros::NodeHandle mPrivateNodeHandle;

  ros::Subscriber mScanSub;

  ros::Publisher mFilteredPointsPub;
  ros::Publisher mFilterInfoPub;

  void ScanCallback(const sensor_msgs::PointCloud2::ConstPtr& input);

  std::string mInputTopic;
  std::string mOutputTopic;

  double mVoxelLeafSize;
  double mMeasurementRange;

  itri_msgs::VoxelGridFilterInfo mVoxelGridFilterInfoMsg;

  std::chrono::time_point<std::chrono::system_clock> mFilterStart;
  std::chrono::time_point<std::chrono::system_clock> mFilterEnd;
};

#endif // VOXEL_GRID_FILTER_H
