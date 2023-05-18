#include <pointmap_loader.h>

namespace {

sensor_msgs::PointCloud2 DownsampleWithRatioSampler(
  sensor_msgs::PointCloud2& cloudMsg, unsigned int targetSize)
{
  if (targetSize > cloudMsg.width)
  {
    ROS_WARN_STREAM("[" << ros::this_node::getName() << ":" << __LINE__
      << "]: targetSize " << targetSize << " is larger than cloudMsg.width "
      << cloudMsg.width << ". Setting targetSize = cloudMsgs.width / 2");
    targetSize = cloudMsg.width / 2;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloudMsg, *cloudPtr);

  const unsigned int stepSize = cloudPtr->size() / targetSize;

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloudPtr(
    new pcl::PointCloud<pcl::PointXYZ>);
  for (auto i=0u; i < cloudPtr->size(); i += stepSize)
  {
    downsampledCloudPtr->push_back(cloudPtr->at(i));
  }

  sensor_msgs::PointCloud2 downsampledCloudMsg;
  pcl::toROSMsg(*downsampledCloudPtr, downsampledCloudMsg);

  return downsampledCloudMsg;
}

} // namespace

PointMapLoader::PointMapLoader()
    : mPrivateNh("~")
    , mDynamicLoading(true)
    , mInitializeWithGnss(false)
    , mInitializeStat(false)
    , mNdtScoreState(false)
    , mIsGnssMsg(false)
    , mMapFileDirectory("/home/d300/Desktop/july_demo_pcd_map")
    , mMapFiles({"submap_0_0.pcd"})
    , mCurrentPubMap({"0_0.pcd"})
    , mNdtScoreThreshold(100)
    , mRadius(0)
    , mRvizDownsampleSize(0u)
{
    InitROS();
    if (!mInitializeWithGnss)
    {
        GetMapFile();
        CreateMap();
    }
}

void PointMapLoader::InitROS()
{
    mPrivateNh.getParam("dynamic_loading", mDynamicLoading);
    mPrivateNh.getParam("map_file_directory", mMapFileDirectory);
    mPrivateNh.getParam("initialize_with_gnss", mInitializeWithGnss);
    mPrivateNh.getParam("ndt_score_threshold", mNdtScoreThreshold);
    mPrivateNh.getParam("search_radius", mRadius);
    int rvizDownsampleSize;
    mPrivateNh.getParam("rviz_downsample_size", rvizDownsampleSize);
    mRvizDownsampleSize = (unsigned int) rvizDownsampleSize;

    mPubMap = mNh.advertise<sensor_msgs::PointCloud2>("points_map", 1, true);
    mPubMapInfo = mNh.advertise<itri_msgs::PointsMapInfo>("points_map_information", 1, true);
    if (mRvizDownsampleSize > 0u)
    {
      mRvizPubMap = mNh.advertise<sensor_msgs::PointCloud2>(
        "rviz_points_map", 1, true);
    }
    mSubLocalization = mNh.subscribe(
        "ndt_pose", 1000, &PointMapLoader::LocalizationCallBack, this);
    mSubGnss = mNh.subscribe(
        "gnss_pose", 10, &PointMapLoader::GnssCallBack, this);
    mSubNdtStat = mNh.subscribe(
        "ndt_statistics", 10, &PointMapLoader::NdtStatCallBack, this);
    mSubInitialPose = mNh.subscribe(
        "/initialpose", 10, &PointMapLoader::InitialPoseCallback, this);

    LoadMapList(mMapFileDirectory + "/submaps_config.json");
}

void PointMapLoader::GetMapFile()
{
    std::string mapFileInput;
    mPrivateNh.getParam("initial_map_files", mapFileInput);
    boost::erase_all(mapFileInput, " ");
    boost::split(mMapFiles, mapFileInput, boost::is_any_of(","));
}

sensor_msgs::PointCloud2 PointMapLoader::LoadMapFile(
    const std::vector<std::string> &mapFiles)
{
    sensor_msgs::PointCloud2 pointMap, nextPointMap;
    for (const auto & mapFile : mapFiles)
    {
        if (pointMap.width == 0)
        {
            bool isFirstLoadPCDFileFailed =
                pcl::io::loadPCDFile(mMapFileDirectory + "/" + mapFile, pointMap) == -1;

            if (isFirstLoadPCDFileFailed)
            {
                ROS_WARN("load %s failed", mapFile.c_str());
            }
        }
        else
        {
            bool isLoadPCDFileFailed =
                pcl::io::loadPCDFile(mMapFileDirectory + "/" + mapFile, nextPointMap) == -1;

            if (isLoadPCDFileFailed)
            {
                ROS_WARN("load %s failed", mapFile.c_str());
            }

            pointMap.width += nextPointMap.width;
            pointMap.row_step += nextPointMap.row_step;
            pointMap.data.insert(pointMap.data.end(),
                nextPointMap.data.begin(), nextPointMap.data.end());
        }
    }

    return pointMap;
}

void PointMapLoader::PublishPointMap(sensor_msgs::PointCloud2 &pointMap)
{
    if (pointMap.width != 0)
    {
        pointMap.header.frame_id = "map";
        mPubMap.publish(pointMap);
    }
}

void PointMapLoader::PublishRvizPointMap(sensor_msgs::PointCloud2 &rvizPointMap)
{
  if(rvizPointMap.width != 0)
  {
    rvizPointMap.header.frame_id = "map";
    mRvizPubMap.publish(rvizPointMap);
  }
}

void PointMapLoader::CreateMap()
{
    sensor_msgs::PointCloud2 mapPoint;
    mapPoint = LoadMapFile(mMapFiles);
    mCurrentPubMap.clear();
    mCurrentPubMap.assign(mMapFiles.begin(), mMapFiles.end());

    PublishPointMap(mapPoint);
    if (mRvizDownsampleSize > 0u && mapPoint.width > 0)
    {
      sensor_msgs::PointCloud2 downsampledCloudMsg =
        DownsampleWithRatioSampler(mapPoint, mRvizDownsampleSize);
      PublishRvizPointMap(downsampledCloudMsg);
    }
    mInitializeStat = true;
}

void PointMapLoader::CreateMap(const geometry_msgs::Point &point)
{
    sensor_msgs::PointCloud2 mapPoint;
    std::vector<std::string> mapFilesOutput;
    SearchNearestSubmap(point, mapFilesOutput);
    if (mapFilesOutput != mCurrentPubMap)
    {
        mapPoint = LoadMapFile(mapFilesOutput);
        ROS_DEBUG_STREAM("[" << ros::this_node::getName() <<
          "]: Created map of size: " << mapPoint.width);
        mCurrentPubMap.clear();
        mCurrentPubMap.assign(mapFilesOutput.begin(), mapFilesOutput.end());
        PublishPointMap(mapPoint);

        if (mRvizDownsampleSize > 0u && mapPoint.width > 0)
        {
          sensor_msgs::PointCloud2 downsampledCloudMsg =
            DownsampleWithRatioSampler(mapPoint, mRvizDownsampleSize);
          PublishRvizPointMap(downsampledCloudMsg);
        }
    }
}

void PointMapLoader::LocalizationCallBack(const geometry_msgs::PoseStamped &msg)
{
    if ((mDynamicLoading && mInitializeStat && mNdtScoreState) ||
        (mDynamicLoading && mInitializeStat && !mIsGnssMsg))
    {
        ROS_DEBUG("Use Localization Pose");
        CreateMap(msg.pose.position);
    }
}

void PointMapLoader::GnssCallBack(const geometry_msgs::PoseStamped &msg)
{
    mIsGnssMsg = true;
    if (mInitializeWithGnss && !mInitializeStat)
    {
        ROS_DEBUG("Use Gnss Pose");
        CreateMap(msg.pose.position);
        mInitializeStat = true;
    }

    if ((mInitializeStat && mDynamicLoading) && (!mNdtScoreState))
    {
        ROS_DEBUG("Use Gnss Pose");
        CreateMap(msg.pose.position);
    }
}

void PointMapLoader::NdtStatCallBack(const itri_msgs::NdtStatistics &msg)
{
    if (msg.ndt_matching_score < mNdtScoreThreshold)
    {
        mNdtScoreState = true;
    }
    else
    {
        mNdtScoreState = false;
    }
}

void PointMapLoader::InitialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_DEBUG("Use Initial Pose");
    CreateMap(msg->pose.pose.position);
    mInitializeStat = true;
}

void PointMapLoader::SearchNearestSubmap(
    const geometry_msgs::Point &point,
    std::vector<std::string> &outputSubmapFiles)
{
    pcl::PointXYZ searchPoint;
    searchPoint.x = point.x;
    searchPoint.y = point.y;
    searchPoint.z = 0.0;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    mSubmapKdtree.radiusSearch(
        searchPoint, mRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    std::sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());

    for(const auto pointIdx : pointIdxRadiusSearch)
        outputSubmapFiles.push_back(mSubmapFileName[pointIdx]);
}

void PointMapLoader::PublishPointMapInfo(const Json::Value &value)
{
    itri_msgs::PointsMapInfo pointsMapInfoMsg;
    if (!value["num_submaps"].empty() && !value["total_points"].empty())
    {
        pointsMapInfoMsg.num_submaps = value["num_submaps"].asUInt();
        pointsMapInfoMsg.total_points = value["total_points"].asUInt();
    }
    else
    {
        ROS_WARN_STREAM("please check num_submaps or total_points in submaps_config.json");
    }

    if (!value["gps_reference"].empty())
    {
        pointsMapInfoMsg.gps_reference.latitude =
            value["gps_reference"]["latitude"].asDouble();
        pointsMapInfoMsg.gps_reference.longitude =
            value["gps_reference"]["longitude"].asDouble();
        pointsMapInfoMsg.gps_reference.altitude =
            value["gps_reference"]["altitude"].asDouble();
    }
    else
    {
        ROS_WARN_STREAM("please check gps_reference in submaps_config.json");
        pointsMapInfoMsg.gps_reference.latitude = NAN;
        pointsMapInfoMsg.gps_reference.longitude = NAN;
        pointsMapInfoMsg.gps_reference.altitude = NAN;
    }

    mPubMapInfo.publish(pointsMapInfoMsg);
}

void PointMapLoader::LoadMapList(const std::string &fileName)
{
    std::ifstream file;
    file.open(fileName, std::ios::binary);
    if (!file.is_open())
    {
        ROS_ERROR("Load submaps_config.json failed.");
    }

    Json::Reader jsonReader;
    if (!jsonReader.parse(file, mValue))
    {
        ROS_ERROR("pointmap loader: Json parse failed.");
        throw std::runtime_error("pointmap loader: Json parse failed.");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr centroidCloud (new pcl::PointCloud<pcl::PointXYZ>);
    centroidCloud->width = mValue["submaps"].size();
    centroidCloud->height = 1;
    centroidCloud->points.resize(centroidCloud->width * centroidCloud->height);

    int cnt = 0;
    for (const auto & submap : mValue["submaps"])
    {
        mSubmapFileName.push_back(submap["file_name"].asString());

        centroidCloud->points[cnt].x = submap["center_x"].asDouble();
        centroidCloud->points[cnt].y = submap["center_y"].asDouble();
        centroidCloud->points[cnt].z = 0.0;

        cnt++;
    }

    mSubmapKdtree.setInputCloud(centroidCloud); //build Kd tree

    PublishPointMapInfo(mValue);

}
