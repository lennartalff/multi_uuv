#!/usr/bin/env python
from __future__ import print_function
from __future__ import division
import time
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from pymavlink import mavutil
from visualization_msgs.msg import Marker


class Node(object):
    def __init__(self):
        pass


def main():
    ser = mavutil.mavlink_connection("/dev/ttyUSB1", baud=57600)
    while True:
        ser.mav.global_vision_position_estimate_send(
            0, 0, 0, 0, 1.0, 2.0, 3.0, 0
        )
        time.sleep(0.5)


if __name__ == "__main__":
    main()