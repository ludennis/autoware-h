import numpy as np
import bisect


'''
    Pose class to store geometric information
'''
class Pose:
    def __init__(self, timestamp, x=0, y=0, z=0, heading_angle=0):
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z
        self.heading_angle = heading_angle

    @classmethod
    def FromTranslation(cls, timestamp, translation):
        return cls(timestamp, translation[0], translation[1],
          translation[2], 0)

    @classmethod
    def FromTimestampedTranslation(cls, timestamped_translation):
        return cls(timestamped_translation.timestamp,
          timestamped_translation.translation[0],
          timestamped_translation.translation[1],
          timestamped_translation.translation[2], 0)

    def __str__(self):
        return "({}: [{}, {}, {}], {})".format(
          self.timestamp, self.x, self.y, self.z, self.heading_angle)


'''
    Timestamped Translation class to be stored and
      compared in Interpolation Buffer
    self.translation = np.array([x, y, z])
    self.timestamp = in seconds (float)
'''
class TimestampedTranslation:
    def __init__(self, timestamp, translation=np.zeros(3)):
        self.timestamp = timestamp
        self.translation = translation

    def __lt__(self, other):
        return self.timestamp < other.timestamp

    def __gt__(self, other):
        return self.timestamp > other.timestamp

    def __eq__(self, other):
        return self.timestamp == other.timestamp


'''
    A class that stores translation and timestamp with interpolated lookup
'''
class TranslationInterpolationBuffer:
    def __init__(self):
        self.mBuffer = []

    def __str__(self):
        return self.mBuffer

    def Push(self, timestamped_translation):
        if (not self.Empty()):
            assert (
              timestamped_translation.timestamp >= self.GetLatestTime()), \
              "New translation is older than latest"
        self.mBuffer.append(timestamped_translation)

    def GetEarliestTime(self):
        assert (not self.Empty()), "Translation Interpolation Buffer is empty"
        return self.mBuffer[0].timestamp

    def GetLatestTime(self):
        assert (not self.Empty()), "Translation Interpolation Buffer is empty"
        return self.mBuffer[-1].timestamp

    def Empty(self):
        return True if len(self.mBuffer) <= 0 else False

    def Has(self, timestamp):
        return self.GetEarliestTime() <= timestamp and \
          timestamp <= self.GetLatestTime()

    def LookUp(self, timestamp):
        assert (self.Has(timestamp)), "Missing translation at time: {}". \
          format(timestamp)
        start = self.mBuffer[bisect.bisect_left(
          self.mBuffer, TimestampedTranslation(timestamp = timestamp))-1]
        end = self.mBuffer[bisect.bisect_right(
          self.mBuffer, TimestampedTranslation(timestamp = timestamp))]
        if end.timestamp == timestamp:
            return end
        return self.Interpolate(start, end, timestamp).translation

    def Interpolate(self, start, end, timestamp):
        assert(timestamp > start.timestamp)
        assert(timestamp < end.timestamp)
        duration = end.timestamp - start.timestamp
        factor = (timestamp - start.timestamp) / duration
        translation = start.translation + \
          (end.translation - start.translation) * factor
        return TimestampedTranslation(timestamp, translation)
