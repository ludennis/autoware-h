#!/usr/bin/python
from __future__ import division

import sys
import rosbag
import csv
import math
import numpy as np

from localization_analysis import gps_util
from localization_analysis import interpolation_buffer as interp

from sklearn.neighbors import NearestNeighbors

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

'''
    pose_analysis plots the following:
        1. transformed_fix_pose (gps fix transformed to velodyne)
        2. fix_pose (gps fix)
        3. source_pose (poses of localization result)
        4. interpolated_pose (interpolated pose between transformed gps fix)
        5. adds histogram plots of while in straight and turning
        6. adds interval analysis(error interval distance: 0.1m, 0.2m, 0.3m)
'''

transformed_fix_pose_list = []

# Hamura
REF_LATITUDE = 35.7705231
REF_LONGITUDE = 139.3241414

# ARTC
# REF_LATITUDE = 24.0603805555556
# REF_LONGITUDE = 120.384379444444

# ITRI
# REF_LATITUDE = 24.775084704
# REF_LONGITUDE = 121.045888961

RTK_FIX_QUALITY = '4'

TF_GPS_TO_VELODYNE_X, TF_GPS_TO_VELODYNE_Y = 0.45, -0.6
USE_INTERPOLATION_BUFFER = False

INTERPOLATION_INTERVAL_SIZE = 30

def createInterpolationPoints(interpolation_interval_size, pose_list):
    interpolation_pose_list = list()
    extend_points_x, extend_points_y, extend_points_timestamp = [], [], []

    for i in range(len(pose_list)-1):
        interpolated_equation_coeff = \
          np.polyfit([pose_list[i].x, pose_list[i+1].x],
                     [pose_list[i].y, pose_list[i+1].y], 1)
        interpolated_equation = np.poly1d(interpolated_equation_coeff)
        interpolated_points_x = np.linspace(pose_list[i].x, pose_list[i+1].x,
        interpolation_interval_size + 2, endpoint=True)
        interpolated_points_y = interpolated_equation(interpolated_points_x)
        interpolated_points_timestamp = list()

        for i in range(len(interpolated_points_x)):
            start_time = pose_list[i].timestamp
            end_time = pose_list[i+1].timestamp
            duration = end_time - start_time
            interpolated_points_timestamp.append(
              start_time + duration / len(interpolated_points_x) * i)

        for ind in range(len(interpolated_points_x)):
            extend_points_x.append(interpolated_points_x[ind])
            extend_points_y.append(interpolated_points_y[ind])
            extend_points_timestamp.append(interpolated_points_timestamp[ind])

    for index in range(len(extend_points_x)):
        interpolation_pose_list.append(
          interp.Pose(extend_points_timestamp[index], extend_points_x[index], \
                      extend_points_y[index]))

    return interpolation_pose_list

def calculateError(interpolation_pose_list, source_pose_list):
    matched_pose_list = []

    nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(
      zip([pose.x for pose in interpolation_pose_list],
          [pose.y for pose in interpolation_pose_list]))

    querry_list = [[pose.x, pose.y] for pose in source_pose_list]
    distances, indices = nbrs.kneighbors(np.array(querry_list), n_neighbors=1)

    matched_pose_list = \
      [interpolation_pose_list[i] for i in [index for index in indices[:,0]]]

    distance_1D_list = [dist[0] for dist in distances]
    return distance_1D_list, matched_pose_list

def getAverageAndStandardDeviation(distances):
    avg = np.average(distances)
    print("The average distance: {}".format(avg))

    std = np.std(distances)
    print("The standard deviation distance: {}".format(std))
    return avg, std

def createInterpolationBuffer(target_fix_pose_list):
    interpolation_buffer = interp.TranslationInterpolationBuffer()
    for fix_pose in target_fix_pose_list:
        timestamped_translation = interp.TimestampedTranslation(
          timestamp = fix_pose.timestamp,
          translation = np.array([fix_pose.x, fix_pose.y, fix_pose.z]))
        interpolation_buffer.Push(timestamped_translation)
    return interpolation_buffer

def calculateErrorWithInterpolateBuffer(source_pose_list, interpolation_buffer):
    interpolated_pose_list = list()
    distances = list()
    for source_pose in source_pose_list:
        interpolated_pose = interp.Pose.FromTranslation(source_pose.timestamp,
          interpolation_buffer.LookUp(source_pose.timestamp))
        interpolated_pose_list.append(interpolated_pose)
        distances.append(math.hypot(source_pose.x - interpolated_pose.x,
        source_pose.y - interpolated_pose.y))
    return distances, interpolated_pose_list

def GPSToVelodyneX(x, y, heading_angle):
    return TF_GPS_TO_VELODYNE_X * math.cos(heading_angle) - \
      TF_GPS_TO_VELODYNE_Y * math.sin(heading_angle) + x

def GPSToVelodyneY(x, y, heading_angle):
    return TF_GPS_TO_VELODYNE_X * math.sin(heading_angle) + \
      TF_GPS_TO_VELODYNE_Y * math.cos(heading_angle) + y

def getHeadingAngle(x, y, prev_x, prev_y):
    return math.atan2(y - prev_y, x - prev_x)

def getPoseDisplayText(pose_list, index):
    return "pose =>\ntimestamp: {}\npoint ({}, {})\nheading: {}\nindex: {}".format(
      " ".join([str(pose_list[n].timestamp) for n in index["ind"]]),
      " ".join([str(pose_list[n].x) for n in index["ind"]]),
      " ".join([str(pose_list[n].y) for n in index["ind"]]),
      " ".join([str(pose_list[n].heading_angle) for n in index["ind"]]),
      " ".join(list(map(str, index["ind"]))))

def getPoseDisplayTextWithErrorDistance(pose_list, index):
    return "pose =>\ntimestamp: {}\npoint ({}, {})\nheading: {}\nerror distance: {}\nindex: {}".format(
      " ".join([str(pose_list[n].timestamp) for n in index["ind"]]),
      " ".join([str(pose_list[n].x) for n in index["ind"]]),
      " ".join([str(pose_list[n].y) for n in index["ind"]]),
      " ".join([str(pose_list[n].heading_angle) for n in index["ind"]]),
      " ".join([str(error_distances[n]) for n in index["ind"]]),
      " ".join(list(map(str, index["ind"]))))

def intervalAnalysis(source_pose, distances):
    lower_ten_centimeter_pose_list = list()
    ten_to_twenty_centimeter_pose_list = list()
    twenty_to_thirty_centimeter_pose_list = list()
    upper_thirty_centimeter_pose_list = list()

    for index in range(len(distances)):
        if distances[index] < 0.1:
            lower_ten_centimeter_pose_list.append(source_pose[index])
        elif distances[index] >= 0.1 and distances[index] < 0.2:
            ten_to_twenty_centimeter_pose_list.append(source_pose[index])
        elif distances[index] >= 0.2 and distances[index] < 0.3:
            twenty_to_thirty_centimeter_pose_list.append(source_pose[index])
        elif distances[index] > 0.3:
            upper_thirty_centimeter_pose_list.append(source_pose[index])

    print("pose count:\n<0.1m:{}, 0.1m-0.2m: {}, 0.2m-0.3m: {}, >0.3m: {}, source: {}".format(
      len(lower_ten_centimeter_pose_list), len(ten_to_twenty_centimeter_pose_list),
      len(twenty_to_thirty_centimeter_pose_list), len(upper_thirty_centimeter_pose_list),
      len(source_pose)))

    plt.figure()
    plt.xlabel('x(meter)')
    plt.ylabel('y(meter)')
    plt.title("Localization interval analysis of error distance")
    plt.plot([pose.x for pose in lower_ten_centimeter_pose_list],
      [pose.y for pose in lower_ten_centimeter_pose_list], 'b.',
      label='<0.1m({0:.2%})'.format(len(lower_ten_centimeter_pose_list)/len(source_pose)))
    plt.plot([pose.x for pose in ten_to_twenty_centimeter_pose_list],
      [pose.y for pose in ten_to_twenty_centimeter_pose_list], 'g.',
      label='0.1m~0.2m({0:.2%})'.format(len(ten_to_twenty_centimeter_pose_list)/len(source_pose)))
    plt.plot([pose.x for pose in twenty_to_thirty_centimeter_pose_list],
      [pose.y for pose in twenty_to_thirty_centimeter_pose_list], 'm.',
      label='0.2m~0.3m({0:.2%})'.format(len(twenty_to_thirty_centimeter_pose_list)/len(source_pose)))
    plt.plot([pose.x for pose in upper_thirty_centimeter_pose_list],
      [pose.y for pose in upper_thirty_centimeter_pose_list], 'r.',
      label='>0.3m({0:.2%})'.format(len(upper_thirty_centimeter_pose_list)/len(source_pose)))

    # find maximum error point
    max_index = np.argmax(distances, axis=0)
    print("maximum error distance:{}m".format(distances[max_index]))
    plt.plot(source_pose[max_index].x, \
      source_pose[max_index].y, \
      'k*', markersize=10, \
      label='maximum localization error({0:.4}m)'.format(distances[max_index]))

    plt.legend()

def update_annotation(index):
    x, y = source_pose_line.get_data()
    source_pose_annotations.xy = \
      (source_pose_list[index["ind"][0]].x, source_pose_list[index["ind"][0]].y)
    source_pose_text = getPoseDisplayTextWithErrorDistance(source_pose_list, index)
    source_pose_annotations.set_text(source_pose_text)
    source_pose_annotations.get_bbox_patch().set_alpha(0.4)

    x, y = interpolated_pose_line.get_data()
    interpolated_pose_annotations.xy = (
      interpolated_pose_list[index["ind"][0]].x,
      interpolated_pose_list[index["ind"][0]].y)
    interpolated_pose_text = getPoseDisplayText(interpolated_pose_list, index)
    interpolated_pose_annotations.set_text(interpolated_pose_text)
    interpolated_pose_annotations.get_bbox_patch().set_alpha(0.4)

    x, y = transformed_fix_pose_line.get_data()
    transformed_fix_pose_annotations.xy = \
      (transformed_fix_pose_list[index["ind"][0]].x, \
       transformed_fix_pose_list[index["ind"][0]].y)
    transformed_fix_pose_text = \
      getPoseDisplayText(transformed_fix_pose_list, index)
    transformed_fix_pose_annotations.set_text(transformed_fix_pose_text)
    transformed_fix_pose_annotations.get_bbox_patch().set_alpha(0.4)

def hover(event):
    source_pose_annotation_is_visible = source_pose_annotations.get_visible()
    interpolated_pose_annotation_is_visible = \
      interpolated_pose_annotations.get_visible()
    transformed_fix_pose_annotation_is_visible = \
      transformed_fix_pose_annotations.get_visible()
    if event.inaxes == ax:
        source_pose_cont, source_pose_index = source_pose_line.contains(event)
        interpolated_pose_cont, interpolated_pose_index = \
          interpolated_pose_line.contains(event)
        transformed_fix_pose_cont, transformed_fix_pose_index = \
          transformed_fix_pose_line.contains(event)
        if source_pose_cont:
            update_annotation(source_pose_index)
            source_pose_annotations.set_visible(True)
            fig.canvas.draw_idle()
        elif interpolated_pose_cont:
            update_annotation(interpolated_pose_index)
            interpolated_pose_annotations.set_visible(True)
            fig.canvas.draw_idle()
        elif transformed_fix_pose_cont:
            update_annotation(transformed_fix_pose_index)
            transformed_fix_pose_annotations.set_visible(True)
            fig.canvas.draw_idle()
        else:
            if source_pose_annotation_is_visible:
                source_pose_annotations.set_visible(False)
                fig.canvas.draw_idle()
            elif interpolated_pose_annotation_is_visible:
                interpolated_pose_annotations.set_visible(False)
                fig.canvas.draw_idle()
            elif transformed_fix_pose_annotation_is_visible:
                transformed_fix_pose_annotations.set_visible(False)
                fig.canvas.draw_idle()

if __name__ == '__main__':
    if len(sys.argv) < 2:
      print ("usage: rosrun localization_analysis pose_analysis.py [bag_filename]")
      sys.exit()

    bag_filename = sys.argv[1]
    bag = rosbag.Bag(bag_filename)
    print ("Reading bag file: {}".format(bag_filename))

    fix_quality = '0'
    fix_cartesian_list = list()
    predict_pose_list = list()
    ndt_pose_list = list()
    heading_angle = 0.0

    for topic, msg, timestamp in bag.read_messages(topics= \
      ['/predict_pose', '/nmea_sentence', '/fix_vs330', '/ndt_pose']):
        if topic == '/nmea_sentence':
            if msg.sentence.split(',')[0] == '$GPGGA':
                fix_quality = gps_util.get_fix_quality(msg)
            if msg.sentence.split(',')[0] == '$GPHDT':
                if msg.sentence.split(',')[1] != "":
                    heading_angle = \
                      float(msg.sentence.split(',')[1]) * -1.0 * \
                      math.pi / 180.0 + math.pi / 2.0
                else:
                    heading_angle = ""
        if topic == '/predict_pose' and fix_quality == RTK_FIX_QUALITY:
            predict_pose_list.append(
              interp.Pose(timestamp = timestamp.to_sec(), x = msg.pose.position.x,
                   y = msg.pose.position.y, z =msg.pose.position.z,
                   heading_angle = heading_angle))
        if topic == '/ndt_pose' and fix_quality == RTK_FIX_QUALITY:
            ndt_pose_list.append(
              interp.Pose(timestamp = timestamp.to_sec(), x = msg.pose.position.x,
                   y = msg.pose.position.y, z =msg.pose.position.z,
                   heading_angle = heading_angle))
        if topic == '/fix_vs330' and fix_quality == RTK_FIX_QUALITY:
            gps_coord = gps_util.parse_gps_coordinates(msg, RTK_FIX_QUALITY)
            cartesian_coord = gps_util.gps_to_cartesian( \
              gps_coord, gps_util.set_gps_reference(REF_LATITUDE, REF_LONGITUDE))
            fix_cartesian_list.append(
              interp.Pose(timestamp = timestamp.to_sec(), x = cartesian_coord.x,
                   y = cartesian_coord.y, z = cartesian_coord.z,
                   heading_angle = heading_angle))

    # motion filter
    indices_to_delete = []
    kGPSMotionFilterVelocityThreshold = 0.28
    for i in range(len(fix_cartesian_list) - 1):
        distance = math.hypot(fix_cartesian_list[i+1].x - fix_cartesian_list[i].x,
                              fix_cartesian_list[i+1].y - fix_cartesian_list[i].y)
        velocity = distance / (fix_cartesian_list[i+1].timestamp - \
                               fix_cartesian_list[i].timestamp)
        if velocity < kGPSMotionFilterVelocityThreshold:
            indices_to_delete.insert(0, i)
    for index in indices_to_delete:
        del fix_cartesian_list[index]

    kPoseMotionFilterVelocityThreshold = 0.56
    indices_to_delete = []
    for i in range(len(predict_pose_list) - 1):
        distance = math.hypot(predict_pose_list[i+1].x - predict_pose_list[i].x,
                              predict_pose_list[i+1].y - predict_pose_list[i].y)
        if (predict_pose_list[i+1].timestamp - predict_pose_list[i].timestamp <= 0.0):
            velocity = 0.0
        else:
            velocity = distance / (predict_pose_list[i+1].timestamp - \
            predict_pose_list[i].timestamp)
        if velocity < kPoseMotionFilterVelocityThreshold:
            indices_to_delete.insert(0, i)
    for index in indices_to_delete:
        del predict_pose_list[index]

    indices_to_delete = []
    for i in range(len(ndt_pose_list) - 1):
        distance = math.hypot(ndt_pose_list[i+1].x - ndt_pose_list[i].x,
                              ndt_pose_list[i+1].y - ndt_pose_list[i].y)
        if (ndt_pose_list[i+1].timestamp - ndt_pose_list[i].timestamp <= 0.0):
            velocity = 0.0
        else:
            velocity = distance / (ndt_pose_list[i+1].timestamp - \
            ndt_pose_list[i].timestamp)
        if velocity < kPoseMotionFilterVelocityThreshold:
            indices_to_delete.insert(0, i)
    for index in indices_to_delete:
        del ndt_pose_list[index]

    # fill heading for each fix pose
    for i in range(1, len(fix_cartesian_list)):
        if fix_cartesian_list[i].heading_angle == "":
            fix_cartesian_list[i].heading_angle = \
              getHeadingAngle(fix_cartesian_list[i].x, fix_cartesian_list[i].y,
                              fix_cartesian_list[i-1].x, fix_cartesian_list[i-1].y)
    del fix_cartesian_list[0]

    # get heading angle for each predict_pose
    for i in range(1, len(predict_pose_list)):
        if predict_pose_list[i].heading_angle == "":
            predict_pose_list[i].heading_angle = \
              getHeadingAngle(predict_pose_list[i].x, predict_pose_list[i].y,
                              predict_pose_list[i-1].x, predict_pose_list[i-1].y)

    del predict_pose_list[0]

    # get heading angle for each ndt_pose
    for i in range(1, len(ndt_pose_list)):
        if ndt_pose_list[i].heading_angle == "":
            ndt_pose_list[i].heading_angle = \
              getHeadingAngle(ndt_pose_list[i].x, ndt_pose_list[i].y,
                              ndt_pose_list[i-1].x, ndt_pose_list[i-1].y)

    del ndt_pose_list[0]

    # applies static transform from gps to velodyne
    original_fix_cartesian_list = fix_cartesian_list[:]
    for i, pose in enumerate(fix_cartesian_list):
        translated_x = GPSToVelodyneX(pose.x, pose.y, pose.heading_angle)
        translated_y = GPSToVelodyneY(pose.x, pose.y, pose.heading_angle)
        fix_cartesian_list[i].x = translated_x
        fix_cartesian_list[i].y = translated_y
    sorted(fix_cartesian_list, key=lambda x:x.timestamp)

    # assign poses
    transformed_fix_pose_list = fix_cartesian_list
    fix_pose_list = original_fix_cartesian_list
    source_pose_list = predict_pose_list

    # interpolates poses out of gps fix
    if USE_INTERPOLATION_BUFFER:
        interpolation_buffer = createInterpolationBuffer(fix_cartesian_list)
        error_distances, interpolated_pose_list = calculateErrorWithInterpolateBuffer(
        predict_pose_list, interpolation_buffer)
        print ("avg distance: {}".format(np.average(error_distances)))
    else:
        interpolation_points_list = \
          createInterpolationPoints(
            INTERPOLATION_INTERVAL_SIZE, transformed_fix_pose_list)
        error_distances, interpolated_pose_list = \
          calculateError(interpolation_points_list, source_pose_list)
        print("avg distance: {}".format(np.average(error_distances)))

    # plot figures
    fig, ax = plt.subplots()
    plt.xlabel('x(meter)')
    plt.ylabel('y(meter)')
    plt.plot([pose.x for pose in transformed_fix_pose_list], \
             [pose.y for pose in transformed_fix_pose_list], 'b-', \
             label='GPS trajectory')
    plt.plot([pose.x for pose in fix_pose_list], \
             [pose.y for pose in fix_pose_list], 'mv', \
             label='GPS Fix raw data (before TF)')
    for i in range(len(source_pose_list)):
        plt.plot([source_pose_list[i].x, interpolated_pose_list[i].x],\
                 [source_pose_list[i].y, interpolated_pose_list[i].y], 'k-')

    # setup for hovering information
    source_pose_line, = plt.plot([pose.x for pose in source_pose_list], \
                                 [pose.y for pose in source_pose_list], 'r.', \
                                 label='Localization')
    interpolated_pose_line, = plt.plot(
      [pose.x for pose in interpolated_pose_list],
      [pose.y for pose in interpolated_pose_list], 'g.',
      label='RTK Fix GPS')
    transformed_fix_pose_line, = plt.plot(
      [pose.x for pose in transformed_fix_pose_list], \
      [pose.y for pose in transformed_fix_pose_list])

    source_pose_annotations = ax.annotate( \
      "", xy=(0, 0), xytext=(-20, 20), textcoords="offset points",
      bbox=dict(boxstyle="round", fc="w"), arrowprops=dict(arrowstyle="->"))
    interpolated_pose_annotations = ax.annotate( \
      "", xy=(0, 0), xytext=(-20, 20), textcoords="offset points",
      bbox=dict(boxstyle="round", fc="w"), arrowprops=dict(arrowstyle="->"))
    transformed_fix_pose_annotations = ax.annotate( \
      "", xy=(0, 0), xytext=(-20, 20), textcoords="offset points",
      bbox=dict(boxstyle="round", fc="w"), arrowprops=dict(arrowstyle="->"))

    source_pose_annotations.set_visible(False)
    interpolated_pose_annotations.set_visible(False)
    transformed_fix_pose_annotations.set_visible(False)

    fig.canvas.mpl_connect("motion_notify_event", hover)
    plt.legend()

    # plots to show non-turning and turning
    plt.figure()
    plt.xlabel('x(meter)')
    plt.ylabel('y(meter)')
    straight_x_list, straight_y_list = list(), list()
    straight_distance_list = list()
    turning_x_list, turning_y_list = list(), list()
    turning_distance_list = list()
    for i in range(1, len(source_pose_list)):
        if np.abs(source_pose_list[i].heading_angle - source_pose_list[i-1].heading_angle) <= 0.01:
            straight_x_list.append(source_pose_list[i].x)
            straight_y_list.append(source_pose_list[i].y)
            straight_distance_list.append(error_distances[i])
        else:
            turning_x_list.append(source_pose_list[i].x)
            turning_y_list.append(source_pose_list[i].y)
            turning_distance_list.append(error_distances[i])
    plt.plot(straight_x_list, straight_y_list, 'r.', markersize=10)
    plt.plot(turning_x_list, turning_y_list, 'b.', markersize=10)

    # estimate average and standard deviation distance
    print("---Localization Distance Error while in Straight Line---")
    straightMean, straightStd = getAverageAndStandardDeviation(straight_distance_list)
    print("---Localization Distance Error while Turning---")
    turningMean, turningStd = getAverageAndStandardDeviation(turning_distance_list)

    # plot histogram of localization distance in straight line
    plt.figure()
    plt.subplot(1,2,1)
    plt.title("Localization Distance Error while in Straight Line")
    plt.hist(np.array(straight_distance_list), bins=int(len(straight_distance_list)/2))
    plt.axvline(straightMean, color='k', linestyle='dashed', linewidth=1)
    _, ylimStraightMax = plt.ylim()
    plt.text(straightMean + straightMean/10,
         ylimStraightMax - ylimStraightMax/10,
         'Mean: {:.6f}m\nStandard Deviation: {:.6f}m'.format(straightMean,straightStd))

    # plot histogram of localization distance in turning line
    plt.subplot(1,2,2)
    plt.title("Localization Distance Error while Turning")
    plt.hist(np.array(turning_distance_list), bins=int(len(turning_distance_list)/2))
    plt.axvline(np.array(turning_distance_list).mean(), color='k', \
      linestyle='dashed', linewidth=1)
    _, ylimTurningMax = plt.ylim()
    plt.text(turningMean + turningMean/10,
         ylimTurningMax - ylimTurningMax/10,
         'Mean: {:.6f}m\nStandard Deviation: {:.6f}m'.format(turningMean,turningStd))

    # interval analysis, error interval distance: 0.1m, 0.2m, 0.3m
    intervalAnalysis(source_pose_list, error_distances)

    # plot ndt_pose, predict_pose, rtk GPS
    plt.figure()
    plt.plot([pose.x for pose in ndt_pose_list], \
             [pose.y for pose in ndt_pose_list], 'bv-', \
             label='ndt_pose')
    plt.plot([pose.x for pose in source_pose_list], \
             [pose.y for pose in source_pose_list], 'r.-', \
             label='predict_pose')
    plt.plot(
      [pose.x for pose in interpolated_pose_list],
      [pose.y for pose in interpolated_pose_list], 'g.',
      label='RTK Fix GPS')
    plt.legend()

    plt.show()

    bag.close()
