#! /usr/bin/env python

import sys
import rosbag
import matplotlib.pyplot as plt
import numpy as np

from localization_analysis import gps_util

REF_LATITUDE = 24.775084704
REF_LONGITUDE = 121.045888961
RTK_FIX_QUALITY = '4'

'''
    @param:
    pose1 = pose2 = dict{"x", "y", "z"}
'''
def Distance2DBetween(pose1, pose2):
    return np.linalg.norm(
      np.array([pose1["x"] - pose2["x"], pose1["y"] - pose1["y"]]))

if __name__ == '__main__':
    if len(sys.argv) < 3:
      print ("usage: rosrun localization_analysis analyze_bag [bag_filename]" \
             " [distance_error_threshold]")
      sys.exit()

    bag_filename = sys.argv[1]
    distance_error_threshold = float(sys.argv[2])
    bag = rosbag.Bag(bag_filename)
    print ("Reading bag file: {}".format(bag_filename))

    predict_poses = \
      [{"x": float(msg.pose.position.x), "y": float(msg.pose.position.y), "z": float(msg.pose.position.z)} \
       for _, msg, _ in bag.read_messages(topics=['/predict_pose'])]
    ndt_stat_list = \
      [msg for _, msg, _ in bag.read_messages(topics=['/ndt_statistics'])]
    smoothed_poses = \
      [{"x": float(msg.pose.position.x), "y": float(msg.pose.position.y), "z": float(msg.pose.position.z)} \
       for _, msg, _ in bag.read_messages(topics=['/predict_pose_smoothed'])]

    print("len(predict_poses): {}, len(smoothed_poses): {}".format(
      len(predict_poses), len(smoothed_poses)))

    '''
        Make sure predict_poses & smoothed_poses have same length
    '''
    if len(predict_poses) > len(smoothed_poses):
        predict_poses = predict_poses[ : len(smoothed_poses)]
    else:
        smoothed_poses = \
          smoothed_poses[ : len(predict_poses)] 

    print("len(predict_poses): {}, len(smoothed_poses): {}".format(
      len(predict_poses), len(smoothed_poses)))

    assert( len(predict_poses) == len(smoothed_poses) )

    '''
        Get GPS data from bag file
    '''
    fix_quality = '0'
    fix_cartesian_list = list()
    for topic, msg, _ in bag.read_messages(topics=['/nmea_sentence', '/fix_vs330']):
        if topic == '/nmea_sentence':
            fix_quality = gps_util.get_fix_quality(msg)
        if topic == '/fix_vs330' and fix_quality == RTK_FIX_QUALITY:
            gps_coord = gps_util.parse_gps_coordinates(msg, RTK_FIX_QUALITY)
            cartesian_coord = gps_util.gps_to_cartesian( \
              gps_coord, gps_util.set_gps_reference(REF_LATITUDE, REF_LONGITUDE))
            fix_cartesian_list.append(cartesian_coord)

    '''
        Pose Analysis
    '''
    # smoothed pose is always one 'pose' behind
    predict_poses = predict_poses[:-1]
    smoothed_poses = smoothed_poses[1:]
    
    indexed_distances = \
      [(index, Distance2DBetween(pose, smoothed_pose)) \
       for index, (pose, smoothed_pose) in \
       enumerate(zip(predict_poses, smoothed_poses))]

    min_distance_index, min_distance = \
      min(indexed_distances, key = lambda x: x[1])
    print("len(indexed_distances): {}".format(len(indexed_distances)))
    print("min_distance_index: {}, min_distance: {}".format(
      min_distance_index, min_distance))
    print("Average distance between poses and smoothed poses: {}".format(
      np.average([distance for _, distance in indexed_distances])))

    high_error_poses = [predict_poses[i] for i, distance in indexed_distances \
      if distance >= distance_error_threshold]
    high_error_pose_indices = [i for i, distance in indexed_distances \
      if distance >= distance_error_threshold]
    high_error_pose_distances = [distance for _, distance in indexed_distances \
      if distance >= distance_error_threshold]

    print("distance_error_threshold: {}".format(distance_error_threshold))
    print("len(high_error_poses): {}, and their (indices, distances): ({}, {})"
      .format(len(high_error_poses), high_error_pose_indices,
              high_error_pose_distances))

    '''
        Draw all the poses
    '''
    plt.title('Pose Analysis From Bag')
    plt.xlabel('x-coordinates (m)')
    plt.ylabel('y-coordinates (m)')

    plt.plot(
      [pose["x"] for pose in predict_poses],
      [pose["y"] for pose in predict_poses],
      '.r-', markersize=10)

    plt.plot(
      [pose["x"] for pose in smoothed_poses],
      [pose["y"] for pose in smoothed_poses],
      '.b-', markersize=10)

    plt.plot(
      [pose["x"] for pose in high_error_poses],
      [pose["y"] for pose in high_error_poses],
      'g.', markersize=50)

    for index in high_error_pose_indices:
      pose = predict_poses[index]
      smoothed_pose = smoothed_poses[index]
      plt.plot([pose["x"], smoothed_pose["x"]],
               [pose["y"], smoothed_pose["y"]], 'g-', markersize=10)

    plt.plot(
      [fix_cartesian.x for fix_cartesian in fix_cartesian_list],
      [fix_cartesian.y for fix_cartesian in fix_cartesian_list],
      '.k', markersize=10)

    plt.show()

    bag.close()
