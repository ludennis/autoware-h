#!/usr/bin/python

import sys
import rosbag
import matplotlib.pyplot as plt
from localization_analysis import interpolation_buffer as interp

'''
    A plotter for ndt poses and predict poses
'''

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: plot_ndt_and_predct_poses.py [bag_filename]")

    bagFilename = sys.argv[1]
    estimatedPoseList = []
    scanMatchedPoseList = []

    # grabs ndt_pose's and predict_pose's from bag file
    bag = rosbag.Bag(bagFilename)
    print ("Reading bag file: {}".format(bagFilename))

    # plot them
    for topic, msg, timestamp in bag.read_messages(topics= \
      ['/ndt_pose', '/predict_pose']):
        if topic == '/ndt_pose':
            scanMatchedPoseList.append(
              interp.Pose(timestamp=timestamp.to_sec(), x=msg.pose.position.x,
                          y=msg.pose.position.y, z=msg.pose.position.z))
        if topic == '/predict_pose':
            estimatedPoseList.append(
              interp.Pose(timestamp=timestamp.to_sec(), x=msg.pose.position.x,
                          y=msg.pose.position.y, z=msg.pose.position.z))

    bag.close()
    fig, ax = plt.subplots()
    plt.title("Scan Matched Poses vs. Estimated Poses")
    scanMatchedPosesLine, = plt.plot([pose.x for pose in scanMatchedPoseList], \
             [pose.y for pose in scanMatchedPoseList], 'b.-', markersize=15)
    estimatedPosesLine, = plt.plot([pose.x for pose in estimatedPoseList], \
             [pose.y for pose in estimatedPoseList], 'r.-', markersize=15)
    plt.legend([scanMatchedPosesLine, estimatedPosesLine], \
               ["Scan Matched Poses (ndt_pose)", "Estimated Poses (predict_pose)"])
    plt.xlabel("x-coordinates (meter)")
    plt.ylabel("y-coordinates (meter)")
    plt.show()
