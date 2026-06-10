#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import os
import subprocess
import threading

import rospy
from std_msgs.msg import String


class QRJichuTrigger:
    def __init__(self):
        rospy.init_node("qr_jichu_trigger")

        config = rospy.get_param("~qr_inspection", {})
        self.detection_topic = str(config.get("detection_topic", "/inspection/qr_detected"))
        self.script_path = os.path.abspath(
            os.path.expanduser(
                str(config.get("jichu_script", "/home/orangepi/dog_ws/px4_control_realease_3.3/jichu1.py"))
            )
        )
        self.trigger_once = bool(config.get("trigger_once", True))
        self.cooldown_sec = max(0.0, float(config.get("trigger_cooldown_sec", 30.0)))
        self.marker_label = str(config.get("marker_label", "cheak point 1"))
        self.allowed_contents = self.load_allowed_contents(config.get("allowed_contents", ["mountain", "ray", "right"]))

        self.lock = threading.Lock()
        self.process = None
        self.triggered = False
        self.last_trigger_time = rospy.Time(0)

        self.sub = rospy.Subscriber(self.detection_topic, String, self.on_detection, queue_size=10)
        rospy.loginfo(
            "qr_jichu_trigger started. topic=%s script=%s trigger_once=%s cooldown=%.1fs allowed=%s",
            self.detection_topic,
            self.script_path,
            self.trigger_once,
            self.cooldown_sec,
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

    def on_detection(self, msg):
        payload = self.parse_payload(msg.data)
        label = payload.get("label", self.marker_label)
        data = str(payload.get("data", "unknown")).strip()
        x = payload.get("x", None)
        y = payload.get("y", None)

        if data.lower() not in self.allowed_contents:
            rospy.loginfo_throttle(1.0, "%s ignored QR content=%s", label, data)
            return

        with self.lock:
            if self.trigger_once and self.triggered:
                return
            if self.process and self.process.poll() is None:
                return
            now = rospy.Time.now()
            if self.last_trigger_time.to_sec() > 0.0 and (now - self.last_trigger_time).to_sec() < self.cooldown_sec:
                return
            self.last_trigger_time = now
            self.triggered = True

        rospy.loginfo("%s trigger jichu1.py QR content=%s center=(%s,%s)", label, data, x, y)
        self.start_script()

    def parse_payload(self, text):
        try:
            payload = json.loads(text)
            if isinstance(payload, dict):
                return payload
        except Exception:
            pass
        return {"label": self.marker_label, "data": text or "unknown"}

    def start_script(self):
        if not os.path.exists(self.script_path):
            rospy.logerr("jichu1.py not found: %s", self.script_path)
            return

        cmd = ["python3", self.script_path]
        cwd = os.path.dirname(self.script_path)
        try:
            process = subprocess.Popen(cmd, cwd=cwd)
        except Exception as exc:
            rospy.logerr("failed to start jichu1.py: %s", exc)
            return

        with self.lock:
            self.process = process
        rospy.loginfo("jichu1.py started pid=%s", process.pid)
        threading.Thread(target=self.wait_process, args=(process,), daemon=True).start()

    def wait_process(self, process):
        code = process.wait()
        rospy.loginfo("jichu1.py exited pid=%s exit_code=%s", process.pid, code)

    def shutdown(self):
        with self.lock:
            process = self.process
        if process and process.poll() is None:
            rospy.logwarn("node shutdown, terminate jichu1.py pid=%s", process.pid)
            process.terminate()

    def run(self):
        rospy.on_shutdown(self.shutdown)
        rospy.spin()


if __name__ == "__main__":
    try:
        QRJichuTrigger().run()
    except rospy.ROSInterruptException:
        pass