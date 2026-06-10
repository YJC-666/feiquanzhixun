#!/usr/bin/env python3
import functools
import http.server
import json
import os
import socketserver
import threading
import urllib.parse

import rospy


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, runtime_config=None, **kwargs):
        self.runtime_config = runtime_config or {}
        super().__init__(*args, **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path
        if path == "/runtime-config.json":
            payload = json.dumps(self.runtime_config, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        super().do_GET()

    def log_message(self, fmt, *args):
        rospy.loginfo("web_ground_station: " + fmt, *args)


class ReusableThreadingTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def runtime_config():
    return {
        "topics": rospy.get_param("~topics", {}),
        "render": rospy.get_param("~render", {}),
        "motion": rospy.get_param("~motion", {}),
        "camera": rospy.get_param("~camera", {}),
    }


def main():
    rospy.init_node("web_pointcloud_ground_station_server")
    port = int(rospy.get_param("~port", 8080))
    web_root = os.path.abspath(os.path.expanduser(rospy.get_param("~web_root", os.getcwd())))

    if not os.path.isdir(web_root):
        raise RuntimeError("web_root does not exist: %s" % web_root)

    handler = functools.partial(NoCacheHandler, directory=web_root, runtime_config=runtime_config())
    server = ReusableThreadingTCPServer(("0.0.0.0", port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    rospy.loginfo("web_pointcloud_ground_station serving %s at http://0.0.0.0:%d", web_root, port)
    rospy.spin()
    server.shutdown()
    server.server_close()


if __name__ == "__main__":
    main()