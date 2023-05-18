#!/usr/bin/env python2

from geometry_msgs.msg import PoseStamped
import json
import numpy
import rospy
import rostest
import unittest

class Node:

    def __init__(self):
        self.hasErrors = False
        self.load_ground_truth()
        self.ndtPoseSubscriber = rospy.Subscriber(
            "/ndt_pose", PoseStamped, self.ndt_pose_callback)

    def error(self, message):
        self.hasErrors = True
        rospy.logerr(message)

    def load_ground_truth(self):
        filePath = rospy.get_param("~file_path")
        with open(filePath, "r") as poseFile:
            self.ndtPose = json.load(poseFile)
        timestamps = sorted(map(int, self.ndtPose.keys()))
        self.firstTimestamp = timestamps[0]
        self.lastTimestamp = timestamps[-1]
        self.maxPositionError = 0.0
        self.maxOrientationError = 0.0

    def ndt_pose_callback(self, message):
        timestamp = message.header.stamp.to_nsec()
        timestampIsInRange = timestamp > self.firstTimestamp and \
            timestamp < self.lastTimestamp
        if timestampIsInRange:
            if str(timestamp) in self.ndtPose:
                self.verify_ndt_pose(message)
            else:
                self.error("Timestamp is not in ground truth")

    def verify_ndt_pose(self, message):
        timestamp = message.header.stamp.to_nsec()
        truePose = self.ndtPose[str(timestamp)]
        truePosition = numpy.array(truePose["position"])
        trueOrientation = numpy.array(truePose["orientation"])
        testPose = message.pose
        testPosition = numpy.array([
            testPose.position.x,
            testPose.position.y,
            testPose.position.z,
        ])
        testOrientation = numpy.array([
            testPose.orientation.x,
            testPose.orientation.y,
            testPose.orientation.z,
            testPose.orientation.w,
        ])
        positionError = numpy.linalg.norm(
            testPosition - truePosition)
        orientationError = numpy.linalg.norm(
            testOrientation - trueOrientation)
        positionWrong = positionError > rospy.get_param("~position_thresh")
        orientationWrong = \
            orientationError > rospy.get_param("~orientation_thresh")
        if positionWrong or orientationWrong:
            self.error("Pose is not correct. "
                "Position error: {}. "
                "Orientation error: {}."
                .format(positionError, orientationError))

class TestCase(unittest.TestCase):

    def test_verify_ndt_pose(self):
        rospy.init_node("verify_ndt_pose")
        node = Node()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.assertFalse(node.hasErrors)

if __name__ == "__main__":
    rostest.rosrun("sdc", "verify_ndt_pose", TestCase)
