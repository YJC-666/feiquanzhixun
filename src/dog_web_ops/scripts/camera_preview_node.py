#!/usr/bin/env python3
import math
import threading

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image


class CameraPreviewCompressor:
    def __init__(self):
        self.lock = threading.Lock()
        self.max_width = max(80, int(self.get_preview_param("max_width", 320)))
        self.jpeg_quality = int(self.clamp(float(self.get_preview_param("jpeg_quality", 45)), 10, 95))
        self.max_fps = max(0.5, float(self.get_preview_param("max_fps", 8.0)))
        self.default_input_topic = self.get_preview_param("input_topic", "/camera/color/image_raw")
        self.streams = self.load_streams()
        self.publishers = {}
        self.outputs_by_input = {}
        self.last_publish_by_input = {}

        for stream in self.streams:
            output_topic = stream["output_topic"]
            input_topic = stream["input_topic"]
            self.publishers[output_topic] = rospy.Publisher(output_topic, CompressedImage, queue_size=1)
            self.outputs_by_input.setdefault(input_topic, []).append(output_topic)

        for input_topic in sorted(self.outputs_by_input.keys()):
            self.last_publish_by_input[input_topic] = rospy.Time(0)
            rospy.Subscriber(input_topic, Image, self.make_callback(input_topic), queue_size=1, buff_size=2**24)

        rospy.loginfo(
            "camera_preview_compressor started. inputs=%s max_width=%d quality=%d max_fps=%.1f",
            ",".join(sorted(self.outputs_by_input.keys())),
            self.max_width,
            self.jpeg_quality,
            self.max_fps,
        )

    def load_streams(self):
        raw_streams = self.get_preview_param("streams", [])
        streams = []
        if isinstance(raw_streams, list):
            for index, item in enumerate(raw_streams):
                if not isinstance(item, dict):
                    continue
                input_topic = str(item.get("input_topic", self.default_input_topic)).strip()
                output_topic = str(
                    item.get("output_topic", "/web/camera/preview_%d/compressed" % (index + 1))
                ).strip()
                if input_topic and output_topic:
                    streams.append({"input_topic": input_topic, "output_topic": output_topic})
        if streams:
            return streams
        return [
            {
                "input_topic": self.default_input_topic,
                "output_topic": "/web/camera/preview_1/compressed",
            },
            {
                "input_topic": self.default_input_topic,
                "output_topic": "/web/camera/preview_2/compressed",
            },
        ]

    def make_callback(self, input_topic):
        def callback(msg):
            self.on_image(input_topic, msg)

        return callback

    def on_image(self, input_topic, msg):
        now = rospy.Time.now()
        with self.lock:
            last_publish = self.last_publish_by_input.get(input_topic, rospy.Time(0))
            if last_publish.to_sec() > 0.0 and (now - last_publish).to_sec() < 1.0 / self.max_fps:
                return
            self.last_publish_by_input[input_topic] = now
            output_topics = list(self.outputs_by_input.get(input_topic, []))

        if not output_topics:
            return

        try:
            image = self.image_to_bgr(msg)
            preview = self.resize_for_preview(image)
            ok, encoded = cv2.imencode(
                ".jpg",
                preview,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if not ok:
                return
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "camera_preview_compressor failed: %s", exc)
            return

        compressed = CompressedImage()
        compressed.header = msg.header
        compressed.header.stamp = now
        compressed.format = "jpeg"
        compressed.data = encoded.tobytes()
        for output_topic in output_topics:
            self.publishers[output_topic].publish(compressed)

    def image_to_bgr(self, msg):
        encoding = (msg.encoding or "").lower()
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if encoding in ("bgr8", "rgb8"):
            channels = 3
            row = data.reshape((height, step))[:, : width * channels]
            image = row.reshape((height, width, channels))
            if encoding == "rgb8":
                return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image

        if encoding in ("bgra8", "rgba8"):
            channels = 4
            row = data.reshape((height, step))[:, : width * channels]
            image = row.reshape((height, width, channels))
            if encoding == "rgba8":
                return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        if encoding in ("mono8", "8uc1"):
            row = data.reshape((height, step))[:, :width]
            return cv2.cvtColor(row, cv2.COLOR_GRAY2BGR)

        raise ValueError("unsupported image encoding: %s" % msg.encoding)

    def resize_for_preview(self, image):
        height, width = image.shape[:2]
        if width <= self.max_width:
            return image
        scale = float(self.max_width) / float(width)
        target_height = max(1, int(math.floor(height * scale)))
        return cv2.resize(image, (self.max_width, target_height), interpolation=cv2.INTER_AREA)

    @staticmethod
    def get_preview_param(key, default):
        return rospy.get_param("~camera_preview/" + key, default)

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))


def main():
    rospy.init_node("camera_preview_compressor")
    CameraPreviewCompressor()
    rospy.spin()


if __name__ == "__main__":
    main()