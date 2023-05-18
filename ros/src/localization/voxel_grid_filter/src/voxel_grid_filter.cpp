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

/*
 * Refactored class according to file:
 * https://github.com/autowarefoundation/autoware/blob/release/
 *  1.8.1/ros/src/sensing/filters/packages/
 *  points_downsampler/nodes/voxel_grid_filter/voxel_grid_filter.cpp
 */

#include <voxel_grid_filter.h>

auto constexpr kMaxMeasurementRange = 200.0;

VoxelGridFilter::VoxelGridFilter()
 : mPrivateNodeHandle("~")
{
  mPrivateNodeHandle.getParam("voxel_leaf_size", mVoxelLeafSize);
  mPrivateNodeHandle.getParam("input_topic", mInputTopic);
  mPrivateNodeHandle.getParam("output_topic", mOutputTopic);

  mFilteredPointsPub = mNh.advertise<sensor_msgs::PointCloud2>(
    mOutputTopic, 10);
  mFilterInfoPub = mNh.advertise<itri_msgs::VoxelGridFilterInfo>(
    "/voxel_grid_filter/info", 1000);
  mScanSub = mNh.subscribe(mInputTopic , 10, &VoxelGridFilter::ScanCallback, this);

  mMeasurementRange = kMaxMeasurementRange;
}

void VoxelGridFilter::Run()
{
  ros::spin();
}

void VoxelGridFilter::ScanCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::fromROSMsg(*input, scan);

  if(mMeasurementRange != kMaxMeasurementRange)
  {
    scan = RemovePointsByRange(scan, 0, mMeasurementRange);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scanPtr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredScanPtr(new pcl::PointCloud<pcl::PointXYZI>());

  sensor_msgs::PointCloud2 filteredMsg;

  mFilterStart = std::chrono::system_clock::now();

  if (mVoxelLeafSize >= 0.1)
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxelGridFilter;
    voxelGridFilter.setLeafSize(mVoxelLeafSize, mVoxelLeafSize, mVoxelLeafSize);
    voxelGridFilter.setInputCloud(scanPtr);
    voxelGridFilter.filter(*filteredScanPtr);
    pcl::toROSMsg(*filteredScanPtr, filteredMsg);
  }
  else
  {
    pcl::toROSMsg(*scanPtr, filteredMsg);
  }

  mFilterEnd = std::chrono::system_clock::now();

  filteredMsg.header = input->header;
  mFilteredPointsPub.publish(filteredMsg);

  mVoxelGridFilterInfoMsg.header = input->header;
  mVoxelGridFilterInfoMsg.filter_name = "voxel_grid_filter";
  mVoxelGridFilterInfoMsg.measurement_range = mMeasurementRange;
  mVoxelGridFilterInfoMsg.original_points_size = scan.size();

  if (mVoxelLeafSize >= 0.1)
  {
    mVoxelGridFilterInfoMsg.filtered_points_size = filteredScanPtr->size();
  }
  else
  {
    mVoxelGridFilterInfoMsg.filtered_points_size = scanPtr->size();
  }

  mVoxelGridFilterInfoMsg.execution_time =
    std::chrono::duration_cast<std::chrono::microseconds>(mFilterEnd - mFilterStart).count() / 1000.0;
  mFilterInfoPub.publish(mVoxelGridFilterInfoMsg);
}

pcl::PointCloud<pcl::PointXYZI> VoxelGridFilter::RemovePointsByRange(
  pcl::PointCloud<pcl::PointXYZI> scan, double minRange, double maxRange)
{
  pcl::PointCloud<pcl::PointXYZI> narrowedScan;
  narrowedScan.header = scan.header;

  if(minRange >= maxRange)
  {
    ROS_ERROR_ONCE("min_range>=max_range @(%lf, %lf)", minRange, maxRange);
    return scan;
  }

  double squareMinRange = minRange * minRange;
  double squareMaxRange = maxRange * maxRange;

  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
  {
    const pcl::PointXYZI &p = *iter;
    double squareDistance = p.x * p.x + p.y * p.y;

    if(squareMinRange <= squareDistance && squareDistance <= squareMaxRange)
    {
      narrowedScan.points.push_back(p);
    }
  }
  return narrowedScan;
}
