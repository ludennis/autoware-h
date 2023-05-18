#include <gtest/gtest.h>
#include <nmea2pose.h>

class GnssPose
{
    public:
        GnssPose()
        {
            mPose.pose.position.x = 0.1;
            mPose.pose.position.y = 0.1;
            mPose.pose.position.z = 0.1;
            mPose.pose.orientation.x = 1.0;
            mPose.pose.orientation.y = 1.0;
            mPose.pose.orientation.z = 1.0;
            mPose.pose.orientation.w = 0.0;
        }

        void callbackSubGnssPose(const geometry_msgs::PoseStamped &msg)
        {
            std::cout <<"--- callbackSubGnssPose --- \n";
            mPose = msg;
        }

        geometry_msgs::PoseStamped GetPose()
        {
            return mPose;
        }
    protected:
        geometry_msgs::PoseStamped mPose;
};

TEST(Nmea2TfPose, CoordinateTransformation)
{
    // create node
    ros::NodeHandle nh;

    // call node
    Nmea2Pose nmeaNode;

    // publish nmea message
    nmea_msgs::Sentence nmeaMsg, nmeaMsg2;
    nmeaMsg.header.stamp.sec = 1531471067;
    nmeaMsg.header.stamp.nsec = 810997962;
    nmeaMsg.header.frame_id = "gps";
    nmeaMsg.sentence =
        "$GPGGA,"
        "083747.80,2446.50251537,N,"
        "12102.75213830,E,"
        "2,19,0.7,121.276,M,16.397,M,2.8,0129*4D";

    nmeaMsg2.header.stamp.sec = 1531471067;
    nmeaMsg2.header.stamp.nsec = 901233911;
    nmeaMsg2.header.frame_id = "gps";
    nmeaMsg2.sentence =
        "$GPHDT,45.780,T*0B";

    ros::Publisher pubNmea =
        nh.advertise<nmea_msgs::Sentence>("nmea_sentence", 0);
    pubNmea.publish(nmeaMsg);
    pubNmea.publish(nmeaMsg2);

    // subscribe gnss_pose
    GnssPose gnssPose;
    ros::Subscriber subGnssPose = nh.subscribe(
        "gnss_pose",
        0,
        &GnssPose::callbackSubGnssPose,
        &gnssPose);

    ros::Rate rate(100);
    for (int i = 0; i < 100; ++ i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_NEAR(gnssPose.GetPose().pose.position.x, -2.021627, 0.001);
    ASSERT_NEAR(gnssPose.GetPose().pose.position.y, -4.738953, 0.001);
    ASSERT_NEAR(gnssPose.GetPose().pose.position.z, -8.920002, 0.001);
}

TEST(Nmea2TfPose, GetOrientation)
{
    // create node
    ros::NodeHandle nh;

    // call node
    Nmea2Pose nmeaNode;

    // publish nmea message
    nmea_msgs::Sentence nmeaMsg, nmeaMsg2;
    nmeaMsg.header.stamp.sec = 1531471067;
    nmeaMsg.header.stamp.nsec = 810997962;
    nmeaMsg.header.frame_id = "gps";
    nmeaMsg.sentence =
        "$GPGGA,"
        "083747.80,2446.50251537,N,"
        "12102.75213830,E,"
        "2,19,0.7,121.276,M,16.397,M,2.8,0129*4D";

    nmeaMsg2.header.stamp.sec = 1531471067;
    nmeaMsg2.header.stamp.nsec = 901233911;
    nmeaMsg2.header.frame_id = "gps";
    nmeaMsg2.sentence =
        "$GPHDT,45.780,T*0B";

    ros::Publisher pubNmea =
        nh.advertise<nmea_msgs::Sentence>("nmea_sentence", 0);
    pubNmea.publish(nmeaMsg);
    pubNmea.publish(nmeaMsg2);
    pubNmea.publish(nmeaMsg);
    pubNmea.publish(nmeaMsg2);

    // subscribe gnss_pose
    GnssPose gnssPose;
    ros::Subscriber subGnssPose = nh.subscribe(
        "gnss_pose",
        0,
        &GnssPose::callbackSubGnssPose,
        &gnssPose);

    ros::Rate rate(100);
    for (int i = 0; i < 100; ++ i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_NEAR(gnssPose.GetPose().pose.orientation.x, 0.0, 0.001);
    ASSERT_NEAR(gnssPose.GetPose().pose.orientation.y, 0.0, 0.001);
    ASSERT_NEAR(gnssPose.GetPose().pose.orientation.z, 0.376386, 0.001);
    ASSERT_NEAR(gnssPose.GetPose().pose.orientation.w, 0.926463, 0.001);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "nmea2pose_test");
    ros::NodeHandle nodeHandle;
    return RUN_ALL_TESTS();
}
