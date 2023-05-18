#include <objects_processor.h>
#include <cmath>

namespace ObjectsProcessor
{
    static std::string INPUT_TOPIC;
    static std::string OUTPUT_TOPIC;

    ObjectsProcessor::ObjectsProcessor()
    :nodeHandle("~")
    {}

    void ObjectsProcessor::CallBackObjs(
        const msgs::DetectedObjectArrayPtr & inObjects)
    {
        mPubObjects =
            nodeHandle.advertise<itri_msgs::DetectedObjectArray>(OUTPUT_TOPIC,1);

        itri_msgs::DetectedObjectArray outputDetectedObjects;
        outputDetectedObjects.header = inObjects->header;

        for (int i = 0; i < inObjects->objects.size(); i++)
        {
            itri_msgs::DetectedObject outputObj;
            objProcessor(inObjects->objects[i],outputObj);
            //
            if (inObjects->objects[i].fusionSourceId == 2)
            {
                outputObj.id = i;
            }
            else
            {
                outputObj.id = i + 400;
            }
            outputDetectedObjects.objects.push_back(outputObj);
        }

        mPubObjects.publish(outputDetectedObjects);
        PubRadarMarkers(outputDetectedObjects);
    }


    void ObjectsProcessor::objProcessor(
        const msgs::DetectedObject & inObject,
        itri_msgs::DetectedObject & outObject)
    {
        itri_msgs::Polygon polygon;
        float centerX = 0.0f;
        float centerY = 0.0f;
        float centerZ = 0.0f;

        itri_msgs::PointXYZSD point;
        point.x = inObject.bPoint.p0.x;
        point.y = inObject.bPoint.p0.y;
        point.z = inObject.bPoint.p0.z;
        centerX += inObject.bPoint.p0.x;
        centerY += inObject.bPoint.p0.y;
        centerZ += inObject.bPoint.p0.z;
        polygon.points.push_back(point);

        point.x = inObject.bPoint.p1.x;
        point.y = inObject.bPoint.p1.y;
        point.z = inObject.bPoint.p1.z;
        centerX += inObject.bPoint.p1.x;
        centerY += inObject.bPoint.p1.y;
        centerZ += inObject.bPoint.p1.z;
        polygon.points.push_back(point);

        point.x = inObject.bPoint.p2.x;
        point.y = inObject.bPoint.p2.y;
        point.z = inObject.bPoint.p2.z;
        centerX += inObject.bPoint.p2.x;
        centerY += inObject.bPoint.p2.y;
        centerZ += inObject.bPoint.p2.z;
        polygon.points.push_back(point);

        point.x = inObject.bPoint.p3.x;
        point.y = inObject.bPoint.p3.y;
        point.z = inObject.bPoint.p3.z;
        centerX += inObject.bPoint.p3.x;
        centerY += inObject.bPoint.p3.y;
        centerZ += inObject.bPoint.p3.z;
        polygon.points.push_back(point);

        point.x = inObject.bPoint.p4.x;
        point.y = inObject.bPoint.p4.y;
        point.z = inObject.bPoint.p4.z;
        centerX += inObject.bPoint.p4.x;
        centerY += inObject.bPoint.p4.y;
        centerZ += inObject.bPoint.p4.z;
        polygon.points.push_back(point);

        point.x = inObject.bPoint.p5.x;
        point.y = inObject.bPoint.p5.y;
        point.z = inObject.bPoint.p5.z;
        centerX += inObject.bPoint.p5.x;
        centerY += inObject.bPoint.p5.y;
        centerZ += inObject.bPoint.p5.z;
        polygon.points.push_back(point);

        point.x = inObject.bPoint.p6.x;
        point.y = inObject.bPoint.p6.y;
        point.z = inObject.bPoint.p6.z;
        centerX += inObject.bPoint.p6.x;
        centerY += inObject.bPoint.p6.y;
        centerZ += inObject.bPoint.p6.z;
        polygon.points.push_back(point);

        point.x = inObject.bPoint.p7.x;
        point.y = inObject.bPoint.p7.y;
        point.z = inObject.bPoint.p7.z;
        centerX += inObject.bPoint.p7.x;
        centerY += inObject.bPoint.p7.y;
        centerZ += inObject.bPoint.p7.z;
        polygon.points.push_back(point);

        outObject.pose.position.x = centerX / 8.0f;
        outObject.pose.position.y = centerY / 8.0f;
        outObject.pose.position.z = centerZ / 8.0f;

        outObject.velocity.linear.x = 0.0f;
        outObject.velocity.linear.y = 0.0f;

        outObject.convex_hull.polygon = polygon;
    }

    void ObjectsProcessor::PubRadarMarkers(
            const itri_msgs::DetectedObjectArray & outObjects)
    {
        if (!outObjects.objects.size())
            return;

        visualization_msgs::MarkerArray objs;
        for (size_t i = 0; i < outObjects.objects.size(); i++)
        {
            visualization_msgs::Marker obj;
            obj.header.frame_id = "/lidar";
            obj.header.stamp = ros::Time::now();
            obj.ns = "points_and_lines";
            obj.action = visualization_msgs::Marker::ADD;
            obj.pose.orientation.w = 1.0;
            obj.id = outObjects.objects[i].id;
            obj.type = visualization_msgs::Marker::LINE_STRIP;
            obj.scale.x = 0.05;
            obj.color.r = 1.0;
            obj.color.g = 1.0;
            obj.color.b = 1.0;
            obj.color.a = 1.0;
            obj.lifetime = ros::Duration(0.1);

            const int order[] = {0,1,5,4,0};

            geometry_msgs::Point markerPt;
            for (const int j : order)
            {
                markerPt.x = outObjects.objects[i].convex_hull.polygon.points[j].x;
                markerPt.y = outObjects.objects[i].convex_hull.polygon.points[j].y;
                markerPt.z =
                    0.5 * outObjects.objects[i].convex_hull.polygon.points[0].z +
                    0.5 * outObjects.objects[i].convex_hull.polygon.points[3].z;
                obj.points.push_back(markerPt);
            }
            obj.points.push_back(markerPt);
            objs.markers.push_back(obj);
        }
        mPubRadarMarkers.publish(objs);
    }

    void ObjectsProcessor::Run()
    {
        ROS_INFO("Initializing data processor, please wait...");
        nodeHandle.param<std::string>("input_point_topic", INPUT_TOPIC, "/LidarDetection");
        ROS_INFO("Input point_topic: %s", INPUT_TOPIC.c_str());

        nodeHandle.param<std::string>("output_point_topic", OUTPUT_TOPIC, "/mmsl_objs");
        ROS_INFO("output_point_topic: %s", OUTPUT_TOPIC.c_str());

        mSubObjsData = nodeHandle.subscribe(INPUT_TOPIC, 1,
            &ObjectsProcessor::CallBackObjs, this);
        mPubRadarMarkers = nodeHandle.advertise<visualization_msgs::MarkerArray>(
            "objects_processor", 1, true);
        ros::spin ();
    }
}
