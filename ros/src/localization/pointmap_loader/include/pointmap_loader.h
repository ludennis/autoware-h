#ifndef POINTMAP_LOADER_H
#define POINTMAP_LOADER_H

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <itri_msgs/NdtStatistics.h>
#include <itri_msgs/PointsMapInfo.h>
#include <jsoncpp/json/json.h>
#include <math.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <stdlib.h>
#include <string>
#include <time.h>

class PointMapLoader
{
    public:
        PointMapLoader();
        void InitROS();
        void GetMapFile();
        sensor_msgs::PointCloud2 LoadMapFile(
            const std::vector<std::string> &mapFiles);
        void PublishPointMap(sensor_msgs::PointCloud2 &pointMap);
        void PublishPointMapInfo(const Json::Value &value);
        void PublishRvizPointMap(sensor_msgs::PointCloud2 &rvizPointMap);
        void CreateMap();
        void CreateMap(const geometry_msgs::Point &point);
        void LocalizationCallBack(const geometry_msgs::PoseStamped &msg);
        void GnssCallBack(const geometry_msgs::PoseStamped &msg);
        void NdtStatCallBack(const itri_msgs::NdtStatistics &msg);
        void InitialPoseCallback(
            const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

        std::vector<std::string> GetMapFileName();
        void LoadMapList(const std::string &fileName);
        void SearchNearestSubmap(
            const geometry_msgs::Point &point,
            std::vector<std::string> &outputSubmapFiles);

    private:
        ros::NodeHandle mNh;
        ros::NodeHandle mPrivateNh;
        ros::Publisher mPubMap;
        ros::Publisher mPubMapInfo;
        ros::Publisher mRvizPubMap;
        ros::Subscriber mSubLocalization;
        ros::Subscriber mSubGnss;
        ros::Subscriber mSubNdtStat;
        ros::Subscriber mSubReliability;
        ros::Subscriber mSubInitialPose;

        bool mDynamicLoading;
        bool mInitializeWithGnss;
        bool mInitializeStat;
        bool mNdtScoreState;
        bool mIsGnssMsg;

        std::string mMapFileDirectory;
        std::vector<std::string> mMapFiles;
        std::vector<std::string> mCurrentPubMap;
        double mNdtScoreThreshold;

        Json::Value mValue;
        std::vector<std::string> mSubmapFileName;
        pcl::KdTreeFLANN<pcl::PointXYZ> mSubmapKdtree;
        float mRadius;

        unsigned int mRvizDownsampleSize;
};

#endif
