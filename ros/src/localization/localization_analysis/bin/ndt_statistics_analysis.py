#!/usr/bin/python

import matplotlib.pyplot as plt
import os
import rosbag
import sys

'''
    Plots the following:
      1. iterations
      2. ndt matching scores
      3. fitness scores
      4. execution time
'''

def PrintMinAvgMax(items, name):
  print("{}: min = {}, avg = {}, max = {}".format(
    name, min(items), sum(items) / len(items), max(items)))

if __name__ == '__main__':
  # reads a bag file and display ndt statistics
  if len(sys.argv) < 2:
    print ("Usage: rosrun {} [start_sequence] [end_sequence] [bag_filename]"
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

  # plots are aggregated according to each ndt statistics item
  for bagDictionary in bagDictionaries:
    bag = rosbag.Bag(bagDictionary['name'])
    print ("Reading bag file: {}".format(bagDictionary['name']))
    ndtStatisticsMessages = \
      [msg for _, msg, _ in bag.read_messages(topics=['/ndt_statistics']) \
       if startSequence <= msg.header.seq <= endSequence]

    bagDictionary['iterations'] = \
      [msg.iteration for msg in ndtStatisticsMessages]
    bagDictionary['ndtMatchingScore'] = \
      [msg.ndt_matching_score for msg in ndtStatisticsMessages]
    bagDictionary['ndtMatchingScoreWithValidPoints'] = \
      [msg.ndt_matching_score_with_valid_points for msg in ndtStatisticsMessages]
    bagDictionary['executionTime'] = \
      [msg.execution_time for msg in ndtStatisticsMessages]
    bagDictionary['fitnessScore'] = \
      [msg.fitness_score for msg in ndtStatisticsMessages]
    bagDictionary['fitnessScoreWithValidPoints'] = \
      [msg.fitness_score_with_valid_points for msg in ndtStatisticsMessages]
    bagDictionary['sequences'] = \
      [msg.header.seq for msg in ndtStatisticsMessages]
    bagDictionary['timestamp'] = \
      [msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 for msg in ndtStatisticsMessages]

    PrintMinAvgMax(bagDictionary['iterations'], 'iterations')
    PrintMinAvgMax(bagDictionary['ndtMatchingScore'], 'ndtMatchingScore')
    PrintMinAvgMax(bagDictionary['ndtMatchingScoreWithValidPoints'],
      'ndtMatchingScoreWithValidPoints')
    PrintMinAvgMax(bagDictionary['executionTime'], 'executionTime')
    PrintMinAvgMax(bagDictionary['fitnessScore'], 'fitnessScore')
    PrintMinAvgMax(bagDictionary['fitnessScoreWithValidPoints'],
      'fitnessScoreWithValidPoints')
    print('')

  plt.title('Sequnces vs Iterations')
  plt.xlabel('Sequences')
  plt.ylabel('Iterations')
  for bagDictionary in bagDictionaries:
    plt.plot(bagDictionary['sequences'], bagDictionary['iterations'])
  plt.legend([bagDictionary['name'] for bagDictionary in bagDictionaries])

  plt.figure()
  plt.title('Sequnces vs NDT Matching Score')
  plt.xlabel('Sequences')
  plt.ylabel('NDT Matching Score')
  for bagDictionary in bagDictionaries:
    plt.plot(bagDictionary['sequences'], bagDictionary['ndtMatchingScoreWithValidPoints'])
  plt.legend([bagDictionary['name'] for bagDictionary in bagDictionaries])

  plt.figure()
  plt.title('Sequence vs NDT Execution Time')
  plt.xlabel('Sequences')
  plt.ylabel('Execution Time (msec)')
  for bagDictionary in bagDictionaries:
    plt.plot(bagDictionary['sequences'], bagDictionary['executionTime'])
  plt.legend([bagDictionary['name'] for bagDictionary in bagDictionaries])

  plt.figure()
  plt.title('Sequences vs Fitness Score')
  plt.xlabel('Sequences')
  plt.ylabel('Fitness Score (meter)')
  for bagDictionary in bagDictionaries:
    plt.plot(bagDictionary['sequences'], bagDictionary['fitnessScoreWithValidPoints'])
  plt.legend([bagDictionary['name'] for bagDictionary in bagDictionaries])

  plt.show()
