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
 *  gnss_localizer/nodes/nmea2tfpose/nmea2tfpose_core.cpp
 */

#include <nmea2pose.h>

Nmea2Pose::Nmea2Pose()
    : mPrivateNh("~")
    , mDataState(false)
    , mGetHeading(true)
    , mRoll(0)
    , mPitch(0)
    , mYaw(0)
    , mOrientationTime(0)
    , mPositionTime(0)
    , mCurrentTime(0)
    , mOrientationStamp(0)
    , mMapFrame("map")
    , mGPSFrame("gps")
    , mGPSReference(0)
{
    initROS();
    mGeo.SetGPSReference(mGPSReference);
    mPrevPose.pose.position.x = mGeo.x();
    mPrevPose.pose.position.y = mGeo.y();
    mPrevPose.pose.position.z = mGeo.z();
    ROS_INFO("GPS Reference num: %d", mGPSReference);
}

void Nmea2Pose::initROS()
{
    mPrivateNh.getParam("gps_reference", mGPSReference);

    mSub = mNh.subscribe("nmea_sentence",
        100, &Nmea2Pose::callbackFromNmeaSentence, this);
    mPub = mNh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
}
void Nmea2Pose::publishPoseStamped()
{
    mPose.header.frame_id = mMapFrame;
    mPose.header.stamp = mCurrentTime;
    mPose.pose.position.x = mGeo.x();
    mPose.pose.position.y = mGeo.y();
    mPose.pose.position.z = mGeo.z();
    mPose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(mRoll, mPitch, mYaw);
    mPub.publish(mPose);
}

void Nmea2Pose::publishTF()
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(
        mPose.pose.position.x,
        mPose.pose.position.y,
        mPose.pose.position.z));
    tf::Quaternion quaternion;
    quaternion.setRPY(mRoll, mPitch, mYaw);
    transform.setRotation(quaternion);
    mTfBr.sendTransform(tf::StampedTransform(
        transform,
        mCurrentTime,
        mMapFrame,
        mGPSFrame));
}

void Nmea2Pose::createOrientation()
{
    double dt = sqrt(pow(mPose.pose.position.x - mPrevPose.pose.position.x, 2) +
        pow(mPose.pose.position.y - mPrevPose.pose.position.y, 2));
    double passed_time = mPose.header.stamp.toSec() - mPrevPose.header.stamp.toSec();
    double velocity = dt / passed_time;

    ROS_INFO("dt: %f", dt);
    ROS_INFO("passed_time: %f", passed_time);
    ROS_INFO("velocity: %f", velocity);

    double threshold = 1.5; // 1.5 meter/sec
    if (!mGetHeading && velocity > threshold)
    {
        mYaw = atan2(mPose.pose.position.y - mPrevPose.pose.position.y,
        mPose.pose.position.x - mPrevPose.pose.position.x);
        mRoll = 0;
        mPitch = 0;

        ROS_INFO("x_now: %f ,y_now: %f",
            mPose.pose.position.x, mPose.pose.position.y);
        ROS_INFO("x_prev: %f, y_prev: %f",
            mPrevPose.pose.position.x, mPrevPose.pose.position.y);
        ROS_INFO("angle:%f, dx:%f, dy:%f", mYaw * 180 / M_PI,
            mPose.pose.position.x - mPrevPose.pose.position.x,
            mPose.pose.position.y - mPrevPose.pose.position.y);
    }
}

void Nmea2Pose::convert(std::vector<std::string> nmea, ros::Time current_stamp)
{
    try
    {
        if (nmea.at(0).compare(0, 2, "QQ") == 0)
        {
            partQQ(nmea, current_stamp);
        }
        else if (nmea.at(0) == "$PASHR")
        {
            partPASHR(nmea, current_stamp);
        }
        else if(nmea.at(0).compare(3, 3, "GGA") == 0)
        {
            partGGA(nmea, current_stamp);
        }
        else if(nmea.at(0) == "$GPRMC")
        {
            partGPRMC(nmea, current_stamp);
        }
        else if(nmea.at(0) == "$GPHDT")
        {
            partGPHDT(nmea, current_stamp);
        }
    }catch (const std::exception &e)
     {
         ROS_WARN_STREAM("Message is invalid : " << e.what());
     }
}

void Nmea2Pose::callbackFromNmeaSentence(
    const nmea_msgs::Sentence::ConstPtr &msg)
{
    mCurrentTime = msg->header.stamp;
    convert(splitString(msg->sentence), msg->header.stamp);

    double timeout = 0.01;
    if ((fabs(mOrientationStamp.toSec() -
        msg->header.stamp.toSec()) > timeout) && mDataState)
    {
        publishPoseStamped();
        createOrientation();
        publishTF();
        mPrevPose = mPose;
    }
}

void Nmea2Pose::partQQ(std::vector<std::string> &nmea, ros::Time current_stamp)
{
    if (!nmea.at(3).empty() && !nmea.at(4).empty() &&
        !nmea.at(5).empty() && !nmea.at(6).empty())
    {
        mOrientationTime = stod(nmea.at(3));
        mRoll = stod(nmea.at(4)) * M_PI / 180.;
        mPitch = -1 * stod(nmea.at(5)) * M_PI / 180.;
        mYaw = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
        mOrientationStamp = current_stamp;
        mDataState = true;
    }
    else
    {
        mDataState = false;
        ROS_WARN("QQ is empty, please check GPS Sensor.");
    }
    ROS_INFO("QQ is subscribed.");
}

void Nmea2Pose::partPASHR(std::vector<std::string> &nmea, ros::Time current_stamp)
{
    if (!nmea.at(1).empty() && !nmea.at(2).empty() &&
        !nmea.at(4).empty() && !nmea.at(5).empty())
    {
        mOrientationTime = stod(nmea.at(1));
        mRoll = stod(nmea.at(4)) * M_PI / 180.;
        mPitch = -1 * stod(nmea.at(5)) * M_PI / 180.;
        mYaw = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
        mDataState = true;
    }
    else
    {
        mDataState = false;
        ROS_WARN("PASHR is empty, please check GPS Sensor.");
    }
    ROS_INFO("PASHR is subscribed.");
}

void Nmea2Pose::partGGA(std::vector<std::string> &nmea, ros::Time current_stamp)
{
    double gps_state = stod(nmea.at(6));
    ROS_INFO("gps_state: %f", gps_state);
    if (!nmea.at(1).empty() && !nmea.at(2).empty() &&
        !nmea.at(4).empty() && !nmea.at(9).empty() &&
        !nmea.at(11).empty() && gps_state != 0)
    {
        mPositionTime = stod(nmea.at(1));
        const double lat = stod(nmea.at(2));
        const double lon = stod(nmea.at(4));
        const double h = stod(nmea.at(9)) + stod(nmea.at(11));
        mGeo.SetLLHNmeaDegrees(lat, lon, h);
        mDataState = true;
    }
    else
    {
        mDataState = false;
        ROS_WARN("GPGGA is empty, please check GPS Sensor.");
    }
    ROS_INFO("GGA is subscribed.");
}

void Nmea2Pose::partGPRMC(std::vector<std::string> &nmea, ros::Time current_stamp)
{
    if (!nmea.at(1).empty() && !nmea.at(3).empty() &&
        !nmea.at(5).empty())
    {
        mPositionTime = stoi(nmea.at(1));
        const double lat = stod(nmea.at(3));
        const double lon = stod(nmea.at(5));
        const double h = 0.0;
        mGeo.SetLLHNmeaDegrees(lat, lon, h);
        mDataState = true;
    }
    else
    {
        mDataState = false;
        ROS_WARN("GPRMC is empty, please check GPS Sensor.");
    }
    ROS_INFO("GPRMC is subscribed.");
}

void Nmea2Pose::partGPHDT(std::vector<std::string> &nmea, ros::Time current_stamp)
{
    mRoll = 0;
    mPitch = 0;
    if (!nmea.at(1).empty())
    {
        ROS_INFO("Use GPHDT Heading");
        mYaw = -1 * stod(nmea.at(1)) * M_PI / 180. + M_PI / 2;
        mGetHeading = true;
    }
    else
    {
        ROS_INFO("GPHDT Heading is empty");
        mGetHeading = false;
    }
    mOrientationStamp = current_stamp;
    ROS_INFO("GPHDT is subscribed.");
}

std::vector<std::string> Nmea2Pose::splitString(const std::string &string)
{
    std::vector<std::string> str_vector;
    std::string tmp_str;
    std::stringstream ss(string);

    while (getline(ss, tmp_str, ','))
      str_vector.push_back(tmp_str);

    return str_vector;
}
