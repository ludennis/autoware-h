#!/usr/bin/env python2
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt


firstReceivedMsg = False
prevPose = PoseStamped()
yawArray = list()
yawMsgArray = list()
diffArray = list()
countArray = list()
count = 1
sum = 0

def predict_pose_callback(pose_msg):
    global firstReceivedMsg
    global prevPose
    global count
    global sum
    if(not firstReceivedMsg):
        firstReceivedMsg = True
    else:
        yaw = math.atan2(
            pose_msg.pose.position.y - prevPose.pose.position.y,
            pose_msg.pose.position.x - prevPose.pose.position.x)
        yawArray.append(yaw)
        orientation_q = pose_msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (r, p, y) = euler_from_quaternion (orientation_list)
        yawMsgArray.append(y)
        diff = yaw - y
        diffArray.append(diff)
        countArray.append(count)
        count = count + 1
        sum = sum + diff
        print("yawArray: " + str(yaw) + "\n")
        print('yawMsgArray: ' + str(y) + '\n')
        print('diffArray: ' + str(yaw-y) + '\n')

    prevPose = pose_msg;

if __name__ == "__main__":
    rospy.init_node("baselink_velodyne_calibrator")
    rospy.Subscriber("/predict_pose", PoseStamped, predict_pose_callback)

    rospy.spin()
    while rospy.is_shutdown():
        plt.figure()
        plt.plot(countArray, yawMsgArray,'r.-', countArray, yawArray, '.-', countArray, diffArray, 'k.-')
        plt.show()
        avg = sum / count
        print("Finish")
        print("Avg: " + str(avg) + '\n')
        break
