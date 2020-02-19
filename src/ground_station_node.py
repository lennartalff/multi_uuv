#!/usr/bin/env python
from __future__ import print_function
from __future__ import division
import rospy
import serial
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from pymavlink import mavutil
from visualization_msgs.msg import Marker


def get_rosparam(name, default_value):
    value = rospy.get_param(name, default_value)
    rospy.loginfo("[{}] {}={}".format(rospy.get_name(), name, value))
    return value


def set_rosparam(name, value):
    rospy.set_param(name, value)
    rospy.loginfo("[{}] {}={}".format(rospy.get_name(), name, value))


class AuvItem(object):
    def __init__(self, node_id, lost=False):
        self.node_id = node_id
        self.pose = PoseStamped()
        self.lost = lost


class Node(object):
    def __init__(self):
        rospy.init_node("ground_station_node")
        rospy.on_shutdown(self._on_shutdown)
        self.port = None
        self.serial = self._connect_serial()
        self.auv_list = []
        self.lost_timeout = 1.0
        self.bad_data_counter = 0
        self.marker_publisher = rospy.Publisher(
            "~auv_position_marker", Marker, queue_size=1)

    def _connect_serial(self, port="/dev/ttyUSB0", baud=57600):
        while True:
            try:
                serial_connection = mavutil.mavlink_connection(
                    port, baud, source_system=0)
            except serial.SerialException:
                rospy.loginfo("[{}] Could not connect to port '{}'\n"
                              "Keep trying.".format(rospy.get_name(), port))
                rospy.sleep(1.0)
            else:
                rospy.loginfo("[{}] Established connection on port '{}'"
                              "".format(rospy.get_name(), port))
                break
        return serial_connection

    def _append_auv(self, auv):
        index = -1
        already_in_list = False
        for i, vehicle in enumerate(self.auv_list):
            # if auv is already in the list
            if vehicle.node_id == auv.node_id:
                index = i
                already_in_list = True
                break

        if not already_in_list:
            self.auv_list.append(auv)

        return index

    def _update_auv(self, auv, index):
        if self.auv_list[index].lost:
            rospy.loginfo("[{}] Rediscovered AUV {}".format(
                rospy.get_name(), auv.node_id))
        self.auv_list[index] = auv

    def update_auvs_lost_status(self):
        now = rospy.Time.now().to_sec()
        for i, auv in enumerate(self.auv_list):

            # only check AUVs that are not already lost
            if not auv.lost:
                since_last = now - auv.pose.header.stamp.to_sec()
                if since_last >= self.lost_timeout:
                    self.auv_list[i].lost = True
                    rospy.loginfo("[{}] AUV {} seems to be lost".format(
                        rospy.get_name(), auv.node_id))

    def read_serial(self):
        return self.serial.recv_msg()

    def handle_message(self, message):
        if not message:
            return
        if message.get_type() == "BAD_DATA":
            self.bad_data_counter += 1
        elif message.get_type() == "GLOBAL_VISION_POSITION_ESTIMATE":
            src_id = message.get_srcSystem()
            auv = AuvItem(src_id)
            # ignore message time stamp? Probably times are not synchronized
            auv.pose.header.stamp = rospy.Time.now()
            auv.pose.pose.position.x = message.x
            auv.pose.pose.position.y = message.y
            auv.pose.pose.position.z = message.z
            q = quaternion_from_euler(message.roll, message.pitch, message.yaw)
            auv.pose.pose.orientation.w = q[0]
            auv.pose.pose.orientation.x = q[1]
            auv.pose.pose.orientation.y = q[2]
            auv.pose.pose.orientation.z = q[3]
            index = self._append_auv(auv)
            if not index < 0:
                self._update_auv(auv, index)
            else:
                rospy.loginfo("[{}] Discovered new AUV: {}".format(
                    rospy.get_name(), src_id))
        else:
            rospy.INFO("[{}] Did not expect message type: '{}'".format(
                rospy.get_name(), message.get_type()))

    def _publish_marker(self, auv):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = auv.pose.header.stamp
        marker.ns = "base_station"
        marker.id = auv.node_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = auv.pose.pose
        marker.scale.x = 0.5
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0 if auv.lost else 0.0
        marker.color.g = 0.0 if auv.lost else 1.0
        try:
            self.marker_publisher.publish(marker)
        except:
            pass

    def publish_markers(self):
        for auv in self.auv_list:
            self._publish_marker(auv)

    def _on_shutdown(self):
        rospy.loginfo("[{}] Shutting down".format(rospy.get_name()))
        self.serial.close()


def main():
    node = Node()
    while not rospy.is_shutdown():
        try:
            msg = node.read_serial()
            if msg:
                node.handle_message(msg)
        except Exception as err:
            rospy.logerr(err)
        node.update_auvs_lost_status()
        node.publish_markers()

if __name__ == "__main__":
    main()
