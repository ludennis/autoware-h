#!/usr/bin/env python2

from geometry_msgs.msg import PoseStamped
import json
import rospy

class Node:
    def __init__(self):
        self.ndtPose = dict()
        self.ndtPoseSubscriber = rospy.Subscriber(
            "/ndt_pose", PoseStamped, self.ndt_pose_callback, queue_size=100)
        self.ndtPoseDumpTimer = rospy.Timer(
            rospy.Duration(10), self.ndt_pose_dump_timer_callback)

    def spin(self):
        rospy.spin()

    def ndt_pose_callback(self, message):
        timestamp = message.header.stamp.to_nsec()
        self.ndtPose[timestamp] = {
            "position": [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            "orientation": [
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ]
        }

    def ndt_pose_dump_timer_callback(self, event):
        rospy.loginfo("Dumping poses ...")
        filePath = rospy.get_param("~file_path")
        with open(filePath, "w") as poseFile:
            json.dump(self.ndtPose, poseFile, sort_keys=True, indent=4)

if __name__ == "__main__":
    rospy.init_node("dump_ndt_pose")
    node = Node()
    node.spin()
