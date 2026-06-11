#!/usr/bin/env python3
import functools
import http.server
import json
import math
import os
import socketserver
import threading
import time
import urllib.parse

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64, String


class CameraMjpegStreamer:
    def __init__(self, config):
        self.config = config if isinstance(config, dict) else {}
        self.width = max(1, int(self.config.get("width", self.config.get("target_width", 640))))
        self.height = max(1, int(self.config.get("height", self.config.get("target_height", 480))))
        self.jpeg_quality = int(self.clamp(float(self.config.get("jpeg_quality", 70)), 10, 95))
        self.max_fps = max(0.5, float(self.config.get("max_fps", 10.0)))
        self.condition = threading.Condition()
        self.latest_by_input = {}
        self.last_encode_by_input = {}
        self.sequence_by_input = {}
        self.streams = self.load_streams()
        self.stream_by_path = {stream["mjpeg_path"]: stream for stream in self.streams}

        self.output_publishers = {}
        self.output_by_input = {}
        for stream in self.streams:
            ot = stream.get("output_topic", "")
            if ot:
                self.output_publishers[ot] = rospy.Publisher(ot, CompressedImage, queue_size=1)
                self.output_by_input.setdefault(stream["input_topic"], []).append(ot)

        input_topics = sorted({stream["input_topic"] for stream in self.streams})
        for input_topic in input_topics:
            self.last_encode_by_input[input_topic] = rospy.Time(0)
            self.sequence_by_input[input_topic] = 0
            rospy.Subscriber(input_topic, Image, self.make_callback(input_topic), queue_size=1, buff_size=2**24)

        rospy.loginfo(
            "dog_web_ops camera mjpeg started. inputs=%s size=%dx%d fps=%.1f paths=%s",
            ",".join(input_topics),
            self.width,
            self.height,
            self.max_fps,
            ",".join(sorted(self.stream_by_path.keys())),
        )

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def load_streams(self):
        raw_streams = self.config.get("streams", [])
        default_input_topic = str(self.config.get("input_topic", "/camera/color/image_raw")).strip()
        streams = []
        if isinstance(raw_streams, list):
            for index, item in enumerate(raw_streams):
                if not isinstance(item, dict):
                    continue
                input_topic = str(item.get("input_topic", default_input_topic)).strip()
                mjpeg_path = str(item.get("mjpeg_path", "/camera/preview_%d.mjpg" % (index + 1))).strip()
                output_topic = str(item.get("output_topic", "")).strip()
                if input_topic and mjpeg_path:
                    streams.append({"input_topic": input_topic, "mjpeg_path": self.normalize_path(mjpeg_path), "output_topic": output_topic})
        if streams:
            return streams
        return [
            {"input_topic": default_input_topic, "mjpeg_path": "/camera/preview_1.mjpg"},
            {"input_topic": default_input_topic, "mjpeg_path": "/camera/preview_2.mjpg"},
        ]

    @staticmethod
    def normalize_path(path):
        path = "/" + str(path).strip("/")
        return path or "/camera/preview_1.mjpg"

    def make_callback(self, input_topic):
        def callback(msg):
            self.on_image(input_topic, msg)

        return callback

    def on_image(self, input_topic, msg):
        now = rospy.Time.now()
        with self.condition:
            last_encode = self.last_encode_by_input.get(input_topic, rospy.Time(0))
            if last_encode.to_sec() > 0.0 and (now - last_encode).to_sec() < 1.0 / self.max_fps:
                return
            self.last_encode_by_input[input_topic] = now

        try:
            image = self.image_to_bgr(msg)
            preview = cv2.resize(image, (self.width, self.height), interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode(
                ".jpg",
                preview,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if not ok:
                return
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "dog_web_ops camera mjpeg encode failed: %s", exc)
            return

        with self.condition:
            self.sequence_by_input[input_topic] = self.sequence_by_input.get(input_topic, 0) + 1
            self.latest_by_input[input_topic] = (self.sequence_by_input[input_topic], encoded.tobytes())
            self.condition.notify_all()

        output_topics = self.output_by_input.get(input_topic, [])
        if output_topics:
            compressed = CompressedImage()
            compressed.header = msg.header
            compressed.header.stamp = now
            compressed.format = "jpeg"
            compressed.data = encoded.tobytes()
            for ot in output_topics:
                pub = self.output_publishers.get(ot)
                if pub:
                    pub.publish(compressed)

    @staticmethod
    def image_to_bgr(msg):
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

        if encoding in ("16uc1", "mono16"):
            row = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, step // 2))[:, :width]
            return CameraMjpegStreamer.depth_to_bgr(row.astype(np.float32))

        if encoding == "32fc1":
            row = np.frombuffer(msg.data, dtype=np.float32).reshape((height, step // 4))[:, :width]
            return CameraMjpegStreamer.depth_to_bgr(row)

        raise ValueError("unsupported image encoding: %s" % msg.encoding)

    @staticmethod
    def depth_to_bgr(depth):
        valid = np.isfinite(depth) & (depth > 0)
        if not np.any(valid):
            return np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)

        values = depth[valid]
        low = float(np.percentile(values, 2.0))
        high = float(np.percentile(values, 98.0))
        if high <= low:
            high = low + 1.0

        normalized = np.zeros(depth.shape, dtype=np.float32)
        normalized[valid] = np.clip((depth[valid] - low) / (high - low), 0.0, 1.0)
        gray = (normalized * 255.0).astype(np.uint8)
        colored = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
        colored[~valid] = (0, 0, 0)
        return colored

    def serve_mjpeg(self, path, handler):
        stream = self.stream_by_path.get(path)
        if not stream:
            return False

        input_topic = stream["input_topic"]
        last_sequence = -1
        handler.send_response(200)
        handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        handler.send_header("Pragma", "no-cache")
        handler.end_headers()

        try:
            while not rospy.is_shutdown():
                with self.condition:
                    self.condition.wait_for(
                        lambda: self.latest_by_input.get(input_topic, (last_sequence, None))[0] != last_sequence
                        or rospy.is_shutdown(),
                        timeout=1.0,
                    )
                    sequence, jpeg = self.latest_by_input.get(input_topic, (last_sequence, None))
                if jpeg is None or sequence == last_sequence:
                    continue
                last_sequence = sequence
                handler.wfile.write(b"--frame\r\n")
                handler.wfile.write(b"Content-Type: image/jpeg\r\n")
                handler.wfile.write(b"Content-Length: %d\r\n\r\n" % len(jpeg))
                handler.wfile.write(jpeg)
                handler.wfile.write(b"\r\n")
                handler.wfile.flush()
                time.sleep(max(0.001, 1.0 / self.max_fps * 0.25))
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
        return True


class RoutedNoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, web_root=None, route_roots=None, runtime_config=None, navigation_settings=None, local_avoidance_settings=None, camera_streamer=None, **kwargs):
        self.web_root = os.path.abspath(web_root or os.getcwd())
        self.route_roots = {
            self.normalize_prefix(prefix): os.path.abspath(root)
            for prefix, root in (route_roots or {}).items()
            if root
        }
        self.runtime_config = runtime_config or {}
        self.navigation_settings = navigation_settings
        self.local_avoidance_settings = local_avoidance_settings
        self.camera_streamer = camera_streamer
        super().__init__(*args, directory=self.web_root, **kwargs)

    @staticmethod
    def normalize_prefix(prefix):
        prefix = "/" + str(prefix).strip("/")
        return prefix + "/"

    @staticmethod
    def safe_join(root, rel_path):
        rel_path = urllib.parse.unquote(rel_path).split("?", 1)[0].split("#", 1)[0]
        rel_path = os.path.normpath(rel_path.lstrip("/"))
        if rel_path in (".", ""):
            rel_path = "index.html"
        if rel_path.startswith(".."):
            rel_path = "index.html"
        return os.path.join(root, rel_path)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path
        if self.camera_streamer and self.camera_streamer.serve_mjpeg(path, self):
            return
        if path == "/api/navigation-settings" and self.navigation_settings:
            self.send_json(self.navigation_settings.read())
            return
        if path == "/api/local-avoidance-settings" and self.local_avoidance_settings:
            self.send_json(self.local_avoidance_settings.read())
            return
        if path == "/runtime-config.json":
            self.send_json(self.runtime_config)
            return
        if path == "/pointcloud/runtime-config.json":
            self.send_json(self.runtime_config.get("pointcloud", {}))
            return
        if path == "/octomap/runtime-config.json":
            self.send_json(self.runtime_config)
            return
        super().do_GET()

    def do_POST(self):
        path = urllib.parse.urlparse(self.path).path
        if path == "/api/navigation-settings" and self.navigation_settings:
            store = self.navigation_settings
        elif path == "/api/local-avoidance-settings" and self.local_avoidance_settings:
            store = self.local_avoidance_settings
        else:
            self.send_error(404, "Not Found")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 8192:
                raise ValueError("invalid request body")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("request body must be a JSON object")
            self.send_json(store.save(payload))
        except Exception as exc:
            rospy.logwarn("dog_web_ops settings save failed: %s", exc)
            self.send_json({"error": str(exc)}, status=400)

    def send_json(self, data, status=200):
        payload = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def translate_path(self, path):
        request_path = urllib.parse.urlparse(path).path
        for prefix, root in self.route_roots.items():
            if request_path == prefix[:-1]:
                return os.path.join(root, "index.html")
            if request_path.startswith(prefix):
                return self.safe_join(root, request_path[len(prefix):])
        return self.safe_join(self.web_root, request_path)

    def log_message(self, fmt, *args):
        rospy.loginfo("dog_web_ops: " + fmt, *args)

class NavigationSettingsStore:
    def __init__(self, path, default_goal_tolerance=0.08, tolerance_pub=None):
        self.path = os.path.abspath(os.path.expanduser(path)) if path else ""
        self.default_goal_tolerance = self.sanitize_goal_tolerance(default_goal_tolerance)
        self.tolerance_pub = tolerance_pub
        self.lock = threading.Lock()

    @staticmethod
    def sanitize_goal_tolerance(value):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("goalTolerance must be positive")
        return max(0.01, min(value, 2.0))

    def read(self):
        settings = {"goalTolerance": self.default_goal_tolerance}
        if not self.path or not os.path.exists(self.path):
            return settings
        try:
            with self.lock:
                with open(self.path, "r", encoding="utf-8") as handle:
                    data = json.load(handle)
            if isinstance(data, dict) and "goalTolerance" in data:
                settings["goalTolerance"] = self.sanitize_goal_tolerance(data["goalTolerance"])
        except Exception as exc:
            rospy.logwarn("dog_web_ops failed to read navigation settings %s: %s", self.path, exc)
        return settings

    def save(self, payload):
        settings = self.read()
        if "goalTolerance" in payload:
            settings["goalTolerance"] = self.sanitize_goal_tolerance(payload["goalTolerance"])
        if self.path:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with self.lock:
                with open(self.path, "w", encoding="utf-8") as handle:
                    json.dump(settings, handle, ensure_ascii=False, indent=2, sort_keys=True)
                    handle.write("\n")
        if self.tolerance_pub:
            self.tolerance_pub.publish(Float64(data=settings["goalTolerance"]))
        return settings


class LocalAvoidanceSettingsStore:
    DEFAULTS = {
        "local_planner_enabled": True,
        "stop_without_cloud": True,
        "front_only_obstacles": True,
        "preview_time": 1.2,
        "stop_distance": 0.55,
        "warning_distance": 1.35,
        "rotate_stop_radius": 0.55,
        "rotate_warning_radius": 0.95,
        "trajectory_horizon": 1.8,
        "trajectory_dt": 0.12,
        "trajectory_collision_margin": 0.08,
        "lateral_margin": 0.15,
        "front_min_x": 0.0,
        "clearance_weight": 4.0,
        "heading_weight": 2.2,
        "speed_weight": 1.4,
        "smoothness_weight": 1.2,
        "rotation_escape_weight": 0.6,
        "point_sample_step": 4,
        "max_points": 5000,
        "candidate_linear_scales": [1.0, 0.75, 0.5, 0.25, 0.0],
        "candidate_angular_offsets": [-0.70, -0.45, -0.25, 0.0, 0.25, 0.45, 0.70],
    }
    SCALAR_LIMITS = {
        "preview_time": (0.2, 4.0),
        "stop_distance": (0.05, 2.0),
        "warning_distance": (0.10, 4.0),
        "rotate_stop_radius": (0.05, 2.0),
        "rotate_warning_radius": (0.10, 4.0),
        "trajectory_horizon": (0.4, 4.0),
        "trajectory_dt": (0.05, 0.5),
        "trajectory_collision_margin": (0.0, 0.5),
        "lateral_margin": (0.0, 0.5),
        "front_min_x": (-0.5, 1.0),
        "clearance_weight": (0.0, 20.0),
        "heading_weight": (0.0, 20.0),
        "speed_weight": (0.0, 20.0),
        "smoothness_weight": (0.0, 20.0),
        "rotation_escape_weight": (0.0, 20.0),
    }
    INT_LIMITS = {
        "point_sample_step": (1, 50),
        "max_points": (100, 20000),
    }
    BOOL_KEYS = {"local_planner_enabled", "stop_without_cloud", "front_only_obstacles"}
    LIST_LIMITS = {
        "candidate_linear_scales": (0.0, 1.0),
        "candidate_angular_offsets": (-2.0, 2.0),
    }

    def __init__(self, path, config_pub=None):
        self.path = os.path.abspath(os.path.expanduser(path)) if path else ""
        self.config_pub = config_pub
        self.lock = threading.Lock()

    @staticmethod
    def clamp_number(value, low, high):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("number must be finite")
        return max(low, min(high, value))

    def sanitize(self, payload):
        settings = dict(self.DEFAULTS)
        if not isinstance(payload, dict):
            return settings
        for key in self.BOOL_KEYS:
            if key in payload:
                settings[key] = bool(payload[key])
        for key, (low, high) in self.SCALAR_LIMITS.items():
            if key in payload:
                settings[key] = self.clamp_number(payload[key], low, high)
        for key, (low, high) in self.INT_LIMITS.items():
            if key in payload:
                settings[key] = int(self.clamp_number(payload[key], low, high))
        for key, (low, high) in self.LIST_LIMITS.items():
            if key in payload and isinstance(payload[key], list):
                values = []
                for item in payload[key][:21]:
                    try:
                        values.append(self.clamp_number(item, low, high))
                    except Exception:
                        pass
                if values:
                    settings[key] = values
        return settings

    def read(self):
        if not self.path or not os.path.exists(self.path):
            return self.sanitize({})
        try:
            with self.lock:
                with open(self.path, "r", encoding="utf-8") as handle:
                    return self.sanitize(json.load(handle))
        except Exception as exc:
            rospy.logwarn("dog_web_ops failed to read local avoidance settings %s: %s", self.path, exc)
            return self.sanitize({})

    def save(self, payload):
        settings = self.sanitize(payload)
        if self.path:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with self.lock:
                with open(self.path, "w", encoding="utf-8") as handle:
                    json.dump(settings, handle, ensure_ascii=False, indent=2, sort_keys=True)
                    handle.write("\n")
        self.publish(settings)
        return settings

    def publish(self, settings=None):
        if self.config_pub:
            self.config_pub.publish(String(data=json.dumps(settings or self.read(), ensure_ascii=False, sort_keys=True)))


class ReusableThreadingTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def get_param_dict(name, default=None):
    value = rospy.get_param(name, default or {})
    return value if isinstance(value, dict) else {}


def runtime_config(route_roots, navigation_config=None):
    routes = {key.strip("/"): f"/{key.strip('/')}/" for key in route_roots.keys()}
    topics = get_param_dict("~topics", {})
    navigation_config = navigation_config or {}
    return {
        "rosbridge": get_param_dict("~rosbridge", {"url": "ws://localhost:9090"}),
        "topics": topics,
        "frames": get_param_dict("~frames", {}),
        "joystick": get_param_dict("~joystick", {}),
        "navigation": {
            "settingsApi": "/api/navigation-settings",
            "defaultGoalTolerance": float(navigation_config.get("default_goal_tolerance", 0.08)),
            "goalToleranceTopic": navigation_config.get("goal_tolerance_topic", topics.get("goal_tolerance", "/navigation/goal_tolerance")),
            "controlModeTopic": navigation_config.get("control_mode_topic", topics.get("control_mode", "/navigation/control_mode")),
        },
        "localAvoidance": {
            "settingsApi": "/api/local-avoidance-settings",
            "configTopic": topics.get("local_avoidance_config", "/dog_safety_mux/local_avoidance_config"),
        },
        "routes": routes,
        "pointcloud": {
            "topics": {
                "cloud": topics.get("cloud", "/pointlio/cloud_registered"),
                "odom": topics.get("odom", "/pointlio/odom"),
                "camera": topics.get("camera", "/camera/image/compressed"),
            },
            "render": get_param_dict("~pointcloud/render", {}),
            "motion": get_param_dict("~pointcloud/motion", {}),
            "camera": get_param_dict("~pointcloud/camera", {}),
        },
    }


def main():
    rospy.init_node("dog_web_ops_server")

    server_config = get_param_dict("~server", {})
    port = int(server_config.get("port", rospy.get_param("~port", 8080)))
    bind = str(server_config.get("bind", rospy.get_param("~bind", "0.0.0.0")))
    web_root = os.path.abspath(os.path.expanduser(rospy.get_param("~web_root", os.getcwd())))
    route_roots = {
        key: os.path.abspath(os.path.expanduser(value))
        for key, value in get_param_dict("~routes", {}).items()
    }
    navigation_config = get_param_dict("~navigation", {})
    default_goal_tolerance = float(navigation_config.get("default_goal_tolerance", 0.08))
    settings_path = navigation_config.get(
        "settings_path",
        os.path.join(web_root, "nav_settings.json"),
    )
    tolerance_topic = navigation_config.get(
        "goal_tolerance_topic",
        get_param_dict("~topics", {}).get("goal_tolerance", "/navigation/goal_tolerance"),
    )
    tolerance_pub = rospy.Publisher(tolerance_topic, Float64, queue_size=1, latch=True)
    navigation_settings = NavigationSettingsStore(settings_path, default_goal_tolerance, tolerance_pub)
    tolerance_pub.publish(Float64(data=navigation_settings.read()["goalTolerance"]))

    local_settings_path = rospy.get_param(
        "~local_avoidance/settings_path",
        os.path.join(web_root, "local_avoidance_settings.json"),
    )
    local_avoidance_topic = get_param_dict("~topics", {}).get("local_avoidance_config", "/dog_safety_mux/local_avoidance_config")
    local_avoidance_pub = rospy.Publisher(local_avoidance_topic, String, queue_size=1, latch=True)
    local_avoidance_settings = LocalAvoidanceSettingsStore(local_settings_path, local_avoidance_pub)
    local_avoidance_settings.publish()

    camera_streamer = CameraMjpegStreamer(get_param_dict("~camera_preview", {}))

    if not os.path.isdir(web_root):
        raise RuntimeError("web_root does not exist: %s" % web_root)
    for key, root in route_roots.items():
        if not os.path.isdir(root):
            rospy.logwarn("dog_web_ops route '%s' does not exist: %s", key, root)

    handler = functools.partial(
        RoutedNoCacheHandler,
        web_root=web_root,
        route_roots=route_roots,
        runtime_config=runtime_config(route_roots, navigation_config),
        navigation_settings=navigation_settings,
        local_avoidance_settings=local_avoidance_settings,
        camera_streamer=camera_streamer,
    )
    server = ReusableThreadingTCPServer((bind, port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    rospy.loginfo("dog_web_ops serving %s at http://%s:%d", web_root, bind, port)
    rospy.loginfo("dog_web_ops navigation settings %s, tolerance topic %s", settings_path, tolerance_topic)
    rospy.loginfo("dog_web_ops local avoidance settings %s, config topic %s", local_settings_path, local_avoidance_topic)
    for key, root in route_roots.items():
        rospy.loginfo("dog_web_ops route /%s/ -> %s", key.strip("/"), root)

    rospy.spin()
    server.shutdown()
    server.server_close()


if __name__ == "__main__":
    main()