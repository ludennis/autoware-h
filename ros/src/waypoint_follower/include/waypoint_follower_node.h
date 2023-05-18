#ifndef __WAYPOINT_FOLLOWER_NODE_H__
#define __WAYPOINT_FOLLOWER_NODE_H__

#include <ros/node_handle.h>
#include <vector>
#include <waypoint_follower_node_context.h>

class WaypointFollowerNode
{
public:
    WaypointFollowerNode();
    void MainLoop();

protected:
    bool HaveWaypoints();

private:
    ros::NodeHandle mNode;
    ros::Publisher mPubControlCmd;
    WaypointFollowerNodeContext mContext;
};

#endif // __WAYPOINT_FOLLOWER_NODE_H__
