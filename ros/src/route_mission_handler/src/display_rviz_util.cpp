#include <route_mission_handler/display_rviz_util.h>

std::vector<std_msgs::ColorRGBA> gMarkerColorsMap;
std_msgs::ColorRGBA gIntersectionColor;
std_msgs::ColorRGBA gTrafficLightColor;

static std_msgs::ColorRGBA SET_COLOR(
    const float b_, const float g_, const float r_)
{
    std_msgs::ColorRGBA color;
    color.r = r_;
    color.g = g_;
    color.b = b_;
    color.a = 1.0f;
    return color;
}

std::string GetNamespace(int type)
{
    switch (type)
    {
        case 1:
            return "waypoint";
        break;
        case 2:
            return "lane";
        break;
        case 3:
            return "road_marker";
        break;
        case 4:
            return "road_line";
        break;
        case 5:
            return "navg_road";
        break;
        case 6:
            return "intersection";
        break;
        case 7:
            return "parking_space";
        break;
        case 8:
            return "traffic_light";
        break;
        case 9:
            return "clicked_point";
        break;
        default:
            return "marker_" + std::to_string(type);
    }
}

visualization_msgs::Marker GetTextMarker(
    bool isAdd,
    int type,
    int index,
    float scaleSize,
    std::string text,
    geometry_msgs::Point &position,
    std_msgs::ColorRGBA color)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = GetNamespace(type) + "_text";
    marker.id = index;

    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = isAdd ?
        visualization_msgs::Marker::ADD:
        visualization_msgs::Marker::DELETE;

    marker.scale.x = scaleSize;
    marker.scale.y = scaleSize;
    marker.scale.z = scaleSize;

    marker.color = color;

    if (color.b == 0.5f && color.g == 0.5f && color.r == 0.5f)
    {
        marker.scale.x = 1.2;
        marker.scale.y = 1.2;
        marker.scale.z = 1.2;

        marker.color.r = 210.0f / 255.0f;
        marker.color.g = 160.0f / 255.0f;
        marker.color.b = 210.0f / 255.0f;
    }
    marker.text = text;
    marker.pose.position = position;
    marker.pose.position.z += 0.5;
    marker.pose.position.x += 0.25;
    tf::Quaternion orien(0,0,0,1);
    orien.normalize();
    tf::quaternionTFToMsg(orien, marker.pose.orientation);
    return marker;
}

visualization_msgs::Marker GetMarker(
    bool isAdd,
    int type,
    int index,
    float scaleSize,
    geometry_msgs::Point &position,
    std_msgs::ColorRGBA color)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = GetNamespace(type);
    marker.id = index;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = isAdd ?
        visualization_msgs::Marker::ADD:
        visualization_msgs::Marker::DELETE;

    marker.scale.x = scaleSize;
    marker.scale.y = scaleSize;
    marker.scale.z = scaleSize;

    marker.color = color;

    marker.pose.position = position;
    tf::Quaternion orien(0,0,0,1);
    orien.normalize();
    tf::quaternionTFToMsg(orien, marker.pose.orientation);
    return marker;
}

visualization_msgs::Marker GetBoundaryLineMarker(
    bool isAdd,
    int index,
    std_msgs::ColorRGBA color,
    std::vector<geometry_msgs::Point> &points)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "boundaries_line_strip";
    marker.id = index;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = isAdd ?
        visualization_msgs::Marker::ADD:
        visualization_msgs::Marker::DELETE;
    marker.scale.x = 0.05;
    marker.pose.orientation.w = 1.0;
    marker.color = color;

    for (int i= 0 ; i < points.size(); i++)
    {
        if (i < points.size() - 1)
        {
            geometry_msgs::Point pnt = points.at(i);
            marker.points.push_back(pnt);
            geometry_msgs::Point pnt2 = points.at(i + 1);
            marker.points.push_back(pnt2);
        }
    }
    return marker;
}

std_msgs::ColorRGBA GetTrafficLightColor()
{
    if (gTrafficLightColor.b==0.0f && gTrafficLightColor.g == 0.0f)
    {
          float b = (rand() % 200 + 50) / 255.0f;
          float g = (rand() % 200 + 50) / 255.0f;
          float r = (rand() % 200 + 50) / 255.0f;
          gTrafficLightColor = SET_COLOR(b, g, r);
    }
    return gTrafficLightColor;
}

std_msgs::ColorRGBA GetIntersectionColor()
{
    if (gIntersectionColor.b==0.0f && gIntersectionColor.g == 0.0f)
    {
        float b = (rand() % 200 + 50) / 255.0f;
        float g = (rand() % 200 + 50) / 255.0f;
        float r = (rand() % 200 + 50) / 255.0f;
          gIntersectionColor = SET_COLOR(b, g, r);
    }
    return gIntersectionColor;
}

std_msgs::ColorRGBA GetTypeColor(int type)
{
    if (gMarkerColorsMap.size() == 0)
    {
        for (int i=0; i <= static_cast<int>(LaneType::HSR_STATION); i++)
        {
            float b = (rand() % 200 + 50) / 255.0f;
            float g = (rand() % 200 + 50) / 255.0f;
            float r = (rand() % 200 + 50) / 255.0f;
            gMarkerColorsMap.push_back(SET_COLOR(b, g, r));
        }

    }
    return gMarkerColorsMap.at(type);
}

visualization_msgs::Marker GetLineStripWithColor(
    int index,
    bool isAdd,
    int type,
    std_msgs::ColorRGBA color)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = GetNamespace(type) + "_color_line";
    marker.id = index;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = isAdd ?
      visualization_msgs::Marker::ADD:
      visualization_msgs::Marker::DELETE;
    marker.scale.x = 0.05;

    tf::Quaternion orien(0,0,0,1);
    orien.normalize();
    tf::quaternionTFToMsg(orien, marker.pose.orientation);

    marker.color = color;
    return marker;
}
