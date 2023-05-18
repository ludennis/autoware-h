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
 * Based on file:
 *  https://github.com/autowarefoundation/autoware/blob/release/
 *  1.8.1/ros/src/computing/perception/localization/packages/
 *  gnss_localizer/nodes/nmea2tfpose/nmea2tfpose_core.h
 */

#ifndef NMEA2POSE_CORE_H
#define NMEA2POSE_CORE_H

#include <string>
#include <memory>
// gps
#include <geo_pos_conv.h>
// PCL includes
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
//ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nmea_msgs/Sentence.h>
#include <tf/transform_broadcaster.h>

class Nmea2Pose
{
    using Point = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;
    using TransformationEstimation =
        pcl::registration::TransformationEstimationSVD<Point, Point>;

    public:
        Nmea2Pose();

    private:
        //variables
        ros::NodeHandle mNh;
        ros::NodeHandle mPrivateNh;
        ros::Publisher mPub;
        ros::Subscriber mSub;
        ITRI::GeoPosConv mGeo;
        ITRI::GeoPosConv mPrevGeo;
        bool mDataState;
        bool mGetHeading;
        double mRoll;
        double mPitch;
        double mYaw;
        double mOrientationTime;
        double mPositionTime;
        ros::Time mCurrentTime;
        ros::Time mOrientationStamp;
        tf::TransformBroadcaster mTfBr;
        const std::string mMapFrame;
        const std::string mGPSFrame;
        int mGPSReference;

        // callbacks
        void callbackFromNmeaSentence(
            const nmea_msgs::Sentence::ConstPtr &msg);

        // functions
        void initROS();
        void publishPoseStamped();
        void publishTF();
        void createOrientation();
        void convert(std::vector<std::string> nmea,
            ros::Time current_stamp);
        void partQQ(std::vector<std::string> &nmea, ros::Time current_stamp);
        void partPASHR(std::vector<std::string> &nmea, ros::Time current_stamp);
        void partGGA(std::vector<std::string> &nmea, ros::Time current_stamp);
        void partGPRMC(std::vector<std::string> &nmea, ros::Time current_stamp);
        void partGPHDT(std::vector<std::string> &nmea, ros::Time current_stamp);
        std::vector<std::string> splitString(const std::string &string);


        // gps transformation
        geometry_msgs::PoseStamped mPrevPose;
        geometry_msgs::PoseStamped mPose;
        TransformationEstimation::Matrix4 Transformation;
};

#endif // __ITRI_NMEA2POSE_CORE_H
