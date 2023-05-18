#!/usr/bin/python

import matplotlib.pyplot as plt
import os
import rosbag
import sys

def PrintMinAvgMax(items, name):
  print("{}: min = {}, avg = {}, max = {}".format(
    name, min(items), sum(items) / len(items), max(items)))

if __name__ == '__main__':
  if len(sys.argv) < 2:
    print ("Usage: rosrun {} [start_sequence] [end_sequence] [bag_filename_1] [bag_filename_2] ..."
      .format(os.path.basename(__file__)))

  if sys.argv[1].isdigit():
    startSequence = int(sys.argv[1])
  else:
    startSequence = 0

  if sys.argv[2].isdigit():
    endSequence = int(sys.argv[2])
  else:
    endSequence = sys.maxint

  if not sys.argv[1].isdigit():
    bagDictionaries = [{'name': bagFilename} for bagFilename in sys.argv[1:]]
  elif not sys.argv[2].isdigit():
    bagDictionaries = [{'name': bagFilename} for bagFilename in sys.argv[2:]]
  else:
    bagDictionaries = [{'name': bagFilename} for bagFilename in sys.argv[3:]]

  for bagDictionary in bagDictionaries:
    bag = rosbag.Bag(bagDictionary['name'])
    print ('Reading bag file: {}'.format(bagDictionary['name']))

    voxelGridFilterInfoMessages = \
      [msg for _, msg, _ in bag.read_messages(topics=['/voxel_grid_filter/info']) \
       if startSequence <= msg.header.seq <= endSequence]

    bagDictionary['executionTime'] = \
      [msg.execution_time for msg in voxelGridFilterInfoMessages]
    bagDictionary['sequences'] = \
      [msg.header.seq for msg in voxelGridFilterInfoMessages]

    PrintMinAvgMax(bagDictionary['executionTime'], 'executionTime')
    print('')

  plt.title('Sequence vs Voxel Grid Filter Execution Time')
  plt.xlabel('Sequences')
  plt.ylabel('Execution Time (msec)')
  for bagDictionary in bagDictionaries:
    plt.plot(bagDictionary['sequences'], bagDictionary['executionTime'])
  plt.legend([bagDictionary['name'] for bagDictionary in bagDictionaries])

  plt.show()
