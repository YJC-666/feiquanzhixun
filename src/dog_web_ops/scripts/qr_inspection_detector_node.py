#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
# pyzbar uses zbar for robust QR decode (no QUIRC dependency)
from pyzbar.pyzbar import decode as pyzbar_decode


class QRInspectionDetector:
    def __init__(self):
        rospy.init_node("qr_inspection_detector")

        config = rospy.get_param("~qr_inspection", {})
        self.input_topic = str(config.get("input_topic", "/camera/color/image_raw"))
        self.overlay_topic = str(config.get("overlay_topic", "/camera/color/qr_overlay"))
        self.detection_topic = str(config.get("detection_topic", "/inspection/qr_detected"))
        self.status_topic = str(config.get("status_topic", "/inspection/qr_status"))
        self.marker_label = str(config.get("marker_label", "cheak point 1"))
        self.max_missing_frames = max(1, int(config.get("max_missing_frames", 5)))
        self.publish_tracked = bool(config.get("publish_tracked", False))
        self.allowed_contents = self.load_allowed_contents(config.get("allowed_contents", ["mountain", "ray", "right"]))

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
        )

        self.tracking = False
        self.prev_gray = None
        self.p0 = None
        self.last_qr_data = ""
        self.last_center = None
        self.frames_since_last_detection = 0
        self.last_logged_data = None

        self.overlay_pub = rospy.Publisher(self.overlay_topic, Image, queue_size=1)
        self.detection_pub = rospy.Publisher(self.detection_topic, String, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.image_sub = rospy.Subscriber(
            self.input_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )

        rospy.loginfo(
            "qr_inspection_detector started. input=%s overlay=%s detection=%s label=%s allowed=%s",
            self.input_topic,
            self.overlay_topic,
            self.detection_topic,
            self.marker_label,
            ",".join(sorted(self.allowed_contents)),
        )

    @staticmethod
    def load_allowed_contents(value):
        if isinstance(value, str):
            raw = value.split(",")
        elif isinstance(value, (list, tuple)):
            raw = value
        else:
            raw = []
        allowed = {str(item).strip().lower() for item in raw if str(item).strip()}
        return allowed or {"mountain", "ray", "right"}

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "qr image convert failed: %s", exc)
            return

        gray = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2GRAY)
        detected = self.detect_qr(gray, frame, msg.header)
        if not detected:
            self.track_qr(gray, frame, msg.header)

        try:
            overlay = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            overlay.header = msg.header
            self.overlay_pub.publish(overlay)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "qr overlay publish failed: %s", exc)

    def detect_qr(self, gray, frame, header):
        try:
            qr_items = self.detect_multi(gray)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "qr decode failed: %s", exc)
            qr_items = []

        if not qr_items:
            with self.lock:
                if self.tracking:
                    self.frames_since_last_detection += 1
            return False

        data, points = self.pick_best_item(qr_items)
        center = self.geometry_center(points)
        self.update_tracking(gray, points, data, center)
        self.draw_qr(frame, points, center, data, tracked=False)
        self.publish_status(data, center, tracked=False)
        if self.is_allowed_data(data):
            self.publish_detection(data, center, header, tracked=False)
        return True

    def detect_multi(self, gray):
        """Detect and decode QR codes using pyzbar (zbar library)."""
        # pyzbar works on the full 3-channel or grayscale image
        decoded_objects = pyzbar_decode(gray)
        items = []
        for obj in decoded_objects:
            data = obj.data.decode("utf-8", errors="replace")
            if not obj.polygon or len(obj.polygon) < 4:
                continue
            # Convert polygon to numpy array of shape (4,2)
            pts = np.array([(p.x, p.y) for p in obj.polygon], dtype=np.float32)
            items.append((data, pts))

        if items:
            return items

        # fallback: try on 3-channel bgr (some pyzbar versions prefer color)
        decoded_objects = pyzbar_decode(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))
        for obj in decoded_objects:
            data = obj.data.decode("utf-8", errors="replace")
            if not obj.polygon or len(obj.polygon) < 4:
                continue
            pts = np.array([(p.x, p.y) for p in obj.polygon], dtype=np.float32)
            items.append((data, pts))

        return items

    def pick_best_item(self, qr_items):
        for data, points in qr_items:
            if self.is_allowed_data(data):
                return data, points
        return qr_items[0]

    @staticmethod
    def geometry_center(points):
        return (int(round(float(np.mean(points[:, 0])))), int(round(float(np.mean(points[:, 1])))))

    def update_tracking(self, gray, points, data, center):
        with self.lock:
            self.last_qr_data = data or "unknown"
            self.last_center = center
            self.frames_since_last_detection = 0
            self.prev_gray = gray.copy()
            self.p0 = points.reshape(-1, 1, 2)
            self.tracking = True

    def track_qr(self, gray, frame, header):
        with self.lock:
            tracking = self.tracking
            prev_gray = None if self.prev_gray is None else self.prev_gray.copy()
            p0 = None if self.p0 is None else self.p0.copy()
            data = self.last_qr_data
            missing = self.frames_since_last_detection

        if not tracking or prev_gray is None or p0 is None or missing > self.max_missing_frames:
            self.clear_tracking_if_needed(missing)
            return False

        p1, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0, None, **self.lk_params)
        if p1 is None or st is None:
            self.reset_tracking("qr tracking failed")
            return False

        good_new = p1[st.reshape(-1) == 1]
        if len(good_new) < 4:
            self.reset_tracking("qr tracking points are not enough")
            return False

        points = self.box_points_from_tracking(good_new.reshape(-1, 2))
        center = self.geometry_center(points)
        with self.lock:
            self.prev_gray = gray.copy()
            self.p0 = points.reshape(-1, 1, 2)
            self.last_center = center

        self.draw_qr(frame, points, center, data, tracked=True)
        self.publish_status(data, center, tracked=True)
        if self.publish_tracked and self.is_allowed_data(data):
            self.publish_detection(data, center, header, tracked=True)
        return True

    @staticmethod
    def box_points_from_tracking(points):
        x_min = float(np.min(points[:, 0]))
        y_min = float(np.min(points[:, 1]))
        x_max = float(np.max(points[:, 0]))
        y_max = float(np.max(points[:, 1]))
        return np.array(
            [[x_min, y_min], [x_max, y_min], [x_max, y_max], [x_min, y_max]],
            dtype=np.float32,
        )

    def clear_tracking_if_needed(self, missing):
        if missing > self.max_missing_frames:
            self.reset_tracking("qr tracking timeout")

    def reset_tracking(self, reason):
        with self.lock:
            self.tracking = False
            self.prev_gray = None
            self.p0 = None
            self.last_qr_data = ""
            self.last_center = None
            self.frames_since_last_detection = 0
        rospy.loginfo_throttle(1.0, reason)

    def draw_qr(self, frame, points, center, data, tracked=False):
        pts = points.astype(int).reshape((-1, 1, 2))
        color = (0, 255, 0)
        cv2.polylines(frame, [pts], True, color, 3)
        cv2.circle(frame, center, 6, (0, 0, 255), -1)
        cv2.drawMarker(frame, center, (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(frame, self.marker_label, (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(frame, "QR content: %s" % (data or "unknown"), (10, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        cv2.putText(frame, "Center: %d,%d" % center, (10, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        cv2.putText(frame, "State: %s" % ("tracking" if tracked else "detected"), (10, 126), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        if not self.is_allowed_data(data):
            cv2.putText(frame, "Trigger: ignored", (10, 156), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
        else:
            cv2.putText(frame, "Trigger: allowed", (10, 156), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

    def is_allowed_data(self, data):
        return str(data or "").strip().lower() in self.allowed_contents

    def publish_status(self, data, center, tracked=False):
        status = "%s QR content:%s center:%d,%d state:%s trigger:%s" % (
            self.marker_label,
            data or "unknown",
            center[0],
            center[1],
            "tracking" if tracked else "detected",
            "allowed" if self.is_allowed_data(data) else "ignored",
        )
        self.status_pub.publish(String(data=status))

    def publish_detection(self, data, center, header, tracked=False):
        payload = {
            "label": self.marker_label,
            "data": str(data or "unknown").strip(),
            "x": int(center[0]),
            "y": int(center[1]),
            "tracked": bool(tracked),
            "stamp": float(header.stamp.to_sec()) if header.stamp else rospy.Time.now().to_sec(),
        }
        text = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.detection_pub.publish(String(data=text))

        log_key = (payload["data"], tracked)
        if log_key != self.last_logged_data:
            rospy.loginfo("%s QR content=%s center=(%d,%d)", self.marker_label, payload["data"], center[0], center[1])
            self.last_logged_data = log_key
        else:
            rospy.loginfo_throttle(1.0, "%s QR content=%s center=(%d,%d)", self.marker_label, payload["data"], center[0], center[1])

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        QRInspectionDetector().run()
    except rospy.ROSInterruptException:
        pass