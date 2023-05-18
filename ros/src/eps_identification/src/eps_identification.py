#!/usr/bin/env python2

import rospy
import numpy as np
import matplotlib.pyplot as plt
from itri_msgs.msg import speed_cmd
from itri_msgs.msg import steer_cmd

def plotSignal(time, sig):
    sig = sig.astype(float)
    plt.plot(np.linspace(0,time + 5,len(sig)), sig)
    plt.xlabel('time(sec)')
    plt.show()

def sine(amp, freq, time):
    t = np.linspace(0, float(time), float(time) * 100 + 1)
    signal = float(amp) * np.sin(2 * np.pi * float(freq) * t)
    return signal

def chripSine(amp, targetHz, time):
    t = np.linspace(0, float(time), float(time) * 100 + 1)
    k = 0.5 * float(targetHz) / float(time)
    signal = float(amp) * np.sin(2 * np.pi * k * t**2)
    return signal

def squareWave(amp, freq, time):
    t = np.linspace(0, float(time), float(time) * 100 + 1)
    signal = np.sin(2 * np.pi * float(freq) * t)
    signal = float(amp) * np.sign(signal)
    return signal

def main():
    rospy.init_node('test_eps', anonymous=True)

    rate = rospy.Rate(100)
    speed_publisher = rospy.Publisher('/speed_cmd', speed_cmd, queue_size=0)
    steer_publisher = rospy.Publisher('/steer_cmd', steer_cmd, queue_size=0)

    signal_type = rospy.get_param('~signal_type')
    amplitude = rospy.get_param('~amplitude')
    frequency = rospy.get_param('~frequency')
    duration = rospy.get_param('~duration')

    if 'square_wave' in signal_type:
        steeringCmd = squareWave(amplitude, frequency, duration)
    elif 'sine_wave' in signal_type:
        steeringCmd = sine(amplitude, frequency, duration)
    elif 'chirp_sine' in signal_type:
        steeringCmd = chripSine(amplitude, frequency, duration)
    else:
        rospy.logerr("missing/wrong siganl type!")
        return

    steeringCmd = np.concatenate((np.zeros(300), steeringCmd), axis = 0)
    plotSignal(duration, steeringCmd)

    speedToPub = speed_cmd()
    steerToPub = steer_cmd()
    speedToPub.type = 1
    steerToPub.type = 1
    speedToPub.kph = 5.0
    for cmd in steeringCmd:
        if not rospy.is_shutdown():
            steerToPub.angle = cmd
            steer_publisher.publish(steerToPub)
            speed_publisher.publish(speedToPub)
            rate.sleep()
        else:
            speedToPub.kph = 0.0
            break

    speedToPub.kph = 0.0
    for i in range(100):
        speed_publisher.publish(speedToPub)
        rate.sleep()

    print('end')

if __name__ == '__main__':
    main()
