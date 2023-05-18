'''
    GPS class and utility functions used by gps_coverage_plotter
'''


import math
import numpy as np


'''
    GPS coordinate class
'''
class GPSCoord(object):
    def __init__(self, latitude, longitude, altitude,fix_quality):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.fix_quality = fix_quality

    def __str__(self):
        return 'latitude: {}, longitude: {}, altitude: {}, fix_quality: {}'.format(
            self.latitude, self.longitude, self.altitude, self.fix_quality)


'''
    Cartesian coordinate class
'''
class CartesianCoord(object):
    def __init__(self, x, y, z, fix_quality):
        self.x = x
        self.y = y
        self.z = y
        self.fix_quality = fix_quality

    def __str__(self):
        return 'x: {}, y: {}, z: {}, fix_quality: {}'.format(
            self.x, self.y, self.z, self.fix_quality)

    def get_color(self):
        if self.fix_quality == '5':
            return 'blue'
        elif self.fix_quality == '4':
            return 'green'
        elif self.fix_quality == '2':
            return 'red'
        else:
            return 'black'


'''
    Reads a nmea_sentence and return the fix quality
    1: Uncorrected coordinate
    2: Differentially correct coordinate (e.g. WAAS, DGPS)
    3: unused
    4: RTK Fix coordinate (centimeter precision)
    5: RTK Float (decimeter precision)
'''
def get_fix_quality(nmea_msg):
    items = nmea_msg.sentence.split(',')
    if items[0] == '$GPGGA':
        fix_quality = items[6]
        return fix_quality


'''
    Parses gps coordinates (latitude, longitude, altitude) from fix messages
    Reads a NavSatFix message and integer, returns a GPSCoord.
'''
def parse_gps_coordinates(fix_msg, fix_quality):
    return GPSCoord(fix_msg.latitude, fix_msg.longitude, fix_msg.altitude, fix_quality)


'''
    Set gps reference to be used in the conversion from gps to cartesian coordinates
'''
def set_gps_reference(latitude, longitude):
    return GPSCoord(latitude, longitude, 0, -1)


'''
    Converts a gps coordinate (latitude, longitude, altitude) into a cartesian
    coordinates (x, y, z). Reads in a GPSCoord to be converted and a reference
    gps coordinates. Returns a CartesianCoord.
'''
def gps_to_cartesian(gps_coord, gps_reference):
    a = 6378137.0
    b = 6356752.3142
    e_squared = 1 - math.pow((b/a), 2)

    # location of reference point in radians
    phi = math.radians(gps_reference.latitude)
    lam = math.radians(gps_reference.longitude)
    h = gps_reference.altitude

    # location of data points in radians
    delta_phi = math.radians(gps_coord.latitude) - phi
    delta_lam = math.radians(gps_coord.longitude) - lam
    delta_h = gps_coord.altitude - h

    # some useful definitions
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)
    N = math.sqrt(1 - e_squared * math.pow(sin_phi, 2))

    # transformations
    x = (a / N + h) * cos_phi * delta_lam - \
        ((a * (1 - e_squared)) / math.pow(N,3) + h) * sin_phi * delta_phi * \
        delta_lam + cos_phi * delta_lam * delta_h

    y = (a * (1-e_squared)/math.pow(N, 3) + h) * delta_phi + \
        1.5 * cos_phi * sin_phi * a * e_squared * math.pow(delta_phi, 2) + \
        math.pow(sin_phi, 2) * delta_h * delta_phi + \
        0.5 * sin_phi * cos_phi * (a/N + h) * math.pow(delta_lam, 2)

    z = delta_h - 0.5 * (a - 1.5 * a * e_squared * math.pow(cos_phi, 2) + \
        0.5 * a * e_squared + h) * math.pow(delta_phi, 2) - \
        0.5 * math.pow(cos_phi, 2) * (a / N - h) * math.pow(delta_lam, 2)

    return CartesianCoord(x, y, z, gps_coord.fix_quality)
