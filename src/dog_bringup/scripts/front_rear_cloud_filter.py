#!/usr/bin/env python3
import math
import struct

import rospy
from sensor_msgs.msg import PointCloud2, PointField


_STRUCT_FORMAT = {
    PointField.INT8: "b",
    PointField.UINT8: "B",
    PointField.INT16: "h",
    PointField.UINT16: "H",
    PointField.INT32: "i",
    PointField.UINT32: "I",
    PointField.FLOAT32: "f",
    PointField.FLOAT64: "d",
}


class FrontRearCloudFilter:
    def __init__(self):
        self.input_topic = rospy.get_param("~topics/input_cloud", "/unilidar/cloud")
        self.pointlio_topic = rospy.get_param("~topics/pointlio_cloud", "/unilidar/cloud_pointlio_filtered")
        self.front_topic = rospy.get_param("~topics/front_cloud", "/unilidar/cloud_front_only")

        self.front_axis = str(rospy.get_param("~filter/front_axis", "x")).strip().lower()
        self.front_min = float(rospy.get_param("~filter/front_min", 0.0))
        self.rear_far_distance = max(0.0, float(rospy.get_param("~filter/rear_far_distance", 0.30)))
        self.keep_rear_far_for_pointlio = bool(rospy.get_param("~filter/keep_rear_far_for_pointlio", True))

        self.pointlio_pub = rospy.Publisher(self.pointlio_topic, PointCloud2, queue_size=1)
        self.front_pub = rospy.Publisher(self.front_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(self.input_topic, PointCloud2, self.on_cloud, queue_size=1)

        rospy.loginfo(
            "front_rear_cloud_filter started. input=%s pointlio=%s front=%s axis=%s front_min=%.3f rear_far=%.3f",
            self.input_topic,
            self.pointlio_topic,
            self.front_topic,
            self.front_axis,
            self.front_min,
            self.rear_far_distance,
        )

    def on_cloud(self, msg):
        axis_field = self.field_by_name(msg, self.front_axis)
        if axis_field is None:
            rospy.logwarn_throttle(2.0, "front_rear_cloud_filter: missing axis field '%s'", self.front_axis)
            return

        unpack_axis = self.make_unpacker(msg, axis_field)
        if unpack_axis is None:
            rospy.logwarn_throttle(
                2.0,
                "front_rear_cloud_filter: unsupported datatype for field '%s': %s",
                self.front_axis,
                axis_field.datatype,
            )
            return

        point_step = int(msg.point_step)
        if point_step <= 0:
            return

        point_count = min(int(msg.width) * max(1, int(msg.height)), len(msg.data) // point_step)
        pointlio_chunks = []
        front_chunks = []
        raw = bytes(msg.data)

        for index in range(point_count):
            base = index * point_step
            axis_value = unpack_axis(raw, base + axis_field.offset)
            if not math.isfinite(axis_value):
                continue

            is_front = axis_value >= self.front_min
            rear_depth = self.front_min - axis_value
            is_rear_far = rear_depth > self.rear_far_distance
            chunk = raw[base:base + point_step]

            if is_front:
                pointlio_chunks.append(chunk)
                front_chunks.append(chunk)
            elif self.keep_rear_far_for_pointlio and is_rear_far:
                pointlio_chunks.append(chunk)

        self.pointlio_pub.publish(self.make_cloud_like(msg, pointlio_chunks))
        self.front_pub.publish(self.make_cloud_like(msg, front_chunks))

    @staticmethod
    def field_by_name(msg, name):
        for field in msg.fields:
            if field.name == name:
                return field
        return None

    @staticmethod
    def make_unpacker(msg, field):
        fmt = _STRUCT_FORMAT.get(field.datatype)
        if not fmt:
            return None
        endian = ">" if msg.is_bigendian else "<"
        unpack = struct.Struct(endian + fmt).unpack_from
        return lambda raw, offset: float(unpack(raw, offset)[0])

    @staticmethod
    def make_cloud_like(source, chunks):
        cloud = PointCloud2()
        cloud.header = source.header
        cloud.height = 1
        cloud.width = len(chunks)
        cloud.fields = source.fields
        cloud.is_bigendian = source.is_bigendian
        cloud.point_step = source.point_step
        cloud.row_step = source.point_step * len(chunks)
        cloud.data = b"".join(chunks)
        cloud.is_dense = False
        return cloud


def main():
    rospy.init_node("front_rear_cloud_filter")
    FrontRearCloudFilter()
    rospy.spin()


if __name__ == "__main__":
    main()